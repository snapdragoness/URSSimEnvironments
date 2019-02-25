#include <string>
#include <thread>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <libcuckoo/cuckoohash_map.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <urs_wearable/Action.h>
#include <urs_wearable/AddArea.h>
#include <urs_wearable/AddLocation.h>
#include <urs_wearable/DroneAction.h>
#include <urs_wearable/Feedback.h>
#include <urs_wearable/GetState.h>
#include <urs_wearable/RemoveArea.h>
#include <urs_wearable/RemoveLocation.h>
#include <urs_wearable/SetGoal.h>
#include <urs_wearable/SetPosition.h>

#include "urs_wearable/common.h"
#include "urs_wearable/knowledge_base.h"

// We use uint8_t here to match with the type of 'value' in ObjectDroneID.msg
typedef std::uint8_t drone_id_type;

KnowledgeBase g_kb("urs", "urs_problem");
ros::Publisher g_feedback_pub;
std::string g_uav_ns;

void execute(KnowledgeBase::executor_id_type executor_id, urs_wearable::SetGoal::Request req)
{
  urs_wearable::Feedback feedback;
  feedback.executor_id = executor_id;
  feedback.status = urs_wearable::Feedback::STATUS_PENDING;
  g_feedback_pub.publish(feedback);
  ros_warn("p" + std::to_string(executor_id) + ": PENDING");

  std::vector<std::string> plan;
  g_kb.getPlan(executor_id, req.goal, plan);

  if (!plan.empty())
  {
    std::vector<urs_wearable::Action> actions = g_kb.parsePlan(plan);

    std::vector<std::string>::iterator plan_it = plan.begin();
    std::vector<urs_wearable::Action>::iterator actions_it = actions.begin();

    while (actions_it != actions.end())
    {
      bool require_drone_action = false;
      drone_id_type drone_id;
      urs_wearable::DroneGoal goal;

      std::vector<urs_wearable::Predicate> effects;

      switch (actions_it->type)
      {
        case urs_wearable::Action::TYPE_ASCEND:
        {
          geometry_msgs::Pose pose_to;
          g_kb.loc_table_.loc_map_.find(actions_it->ascend.l1.value, pose_to);

          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->ascend.NAME + "(" + std::to_string(actions_it->ascend.d.value) + ",("
                   + std::to_string(pose_to.position.x) + "," + std::to_string(pose_to.position.y) + "," + std::to_string(pose_to.position.z) + "))");

          require_drone_action = true;
          drone_id = actions_it->ascend.d.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          goal.poses.push_back(pose_to);

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_AT;
          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->ascend.l0.value;
          effect.truth_value = false;
          effects.push_back(effect);

          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->ascend.l1.value;
          effect.truth_value = true;
          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_DESCEND:
        {
          geometry_msgs::Pose pose_to;
          g_kb.loc_table_.loc_map_.find(actions_it->descend.l1.value, pose_to);

          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->descend.NAME + "(" + std::to_string(actions_it->descend.d.value) + ",("
                   + std::to_string(pose_to.position.x) + "," + std::to_string(pose_to.position.y) + "," + std::to_string(pose_to.position.z) + "))");

          require_drone_action = true;
          drone_id = actions_it->descend.d.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          goal.poses.push_back(pose_to);

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_AT;
          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->descend.l0.value;
          effect.truth_value = false;
          effects.push_back(effect);

          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->descend.l1.value;
          effect.truth_value = true;
          effects.push_back(effect);
        }
        break;

        // FIXME
        case urs_wearable::Action::TYPE_GATHER:
        break;

        case urs_wearable::Action::TYPE_LAND:
        {
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->land.NAME + "(" + std::to_string(actions_it->land.d.value) + ")");

          require_drone_action = true;
          drone_id = actions_it->land.d.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_LAND;

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_HOVERED;
          effect.hovered.d.value = actions_it->land.d.value;
          effect.truth_value = false;
          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_MOVE:
        {
          geometry_msgs::Pose pose_to;
          g_kb.loc_table_.loc_map_.find(actions_it->move.l1.value, pose_to);

          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->move.NAME + "(" + std::to_string(actions_it->move.d.value) + ",("
                   + std::to_string(pose_to.position.x) + "," + std::to_string(pose_to.position.y) + "," + std::to_string(pose_to.position.z) + "))");

          // TODO
//          if (!isWithinActiveRegion(pose_to, location_id_to, feedback.message))
//          {
//            ROS_WARN("Executor %u: ABORTED", executor_id);
//            feedback.status = urs_wearable::Feedback::STATUS_ABORTED;
//            feedback_pub.publish(feedback);
//            ros::spinOnce();
//            rate.sleep();
//
//            g_kb.unregisterExecutor(executor_id);
//            return;
//          }

          require_drone_action = true;
          drone_id = actions_it->move.d.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          goal.poses.push_back(pose_to);

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_AT;
          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->move.l0.value;
          effect.truth_value = false;
          effects.push_back(effect);

          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->move.l1.value;
          effect.truth_value = true;
          effects.push_back(effect);
        }
        break;

        // FIXME
        case urs_wearable::Action::TYPE_SCAN:
        {
//          geometry_msgs::Pose pose_to;
//          g_kb.location_table_.map_.find(actions_it->scan.l1.value, pose_to);
//
//          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
//          feedback.current_action = *actions_it;
//          g_feedback_pub.publish(feedback);
//          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
//                   + actions_it->scan.NAME + "(" + std::to_string(actions_it->scan.d.value) + ",("
//                   + std::to_string(pose_to.position.x) + "," + std::to_string(pose_to.position.y) + "," + std::to_string(pose_to.position.z) + "))");
//
//          require_drone_action = true;
//          drone_id = actions_it->scan.d.value;
//          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
//          goal.poses.push_back(pose_to);
//
//          // Add the effects of the action to the list
//          urs_wearable::Predicate effect;
//          effect.type = urs_wearable::Predicate::TYPE_AT;
//          effect.at.d.value = drone_id;
//          effect.at.l.value = actions_it->scan.l0.value;
//          effect.at.truth_value = false;
//          effects.push_back(effect);
//
//          effect.at.d.value = drone_id;
//          effect.at.l.value = actions_it->scan.l1.value;
//          effect.at.truth_value = true;
//          effects.push_back(effect);
//
//          effect.type = urs_wearable::Predicate::TYPE_SCANNED;
//          effect.scanned.l.value = actions_it->scan.l1.value;
//          effect.scanned.truth_value = true;
//          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_TAKEOFF:
        {
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->takeoff.NAME + "(" + std::to_string(actions_it->takeoff.d.value) + ")");

          require_drone_action = true;
          drone_id = actions_it->takeoff.d.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_TAKEOFF;

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_HOVERED;
          effect.hovered.d.value = actions_it->takeoff.d.value;
          effect.truth_value = true;
          effects.push_back(effect);
        }
        break;

        default:
        {
          ros_error("p" + std::to_string(executor_id) + ": unrecognized action");
          g_kb.unregisterExecutor(executor_id);
          return;
        }
      }

      if (require_drone_action)
      {
        actionlib::SimpleActionClient<urs_wearable::DroneAction> ac("/uav" + std::to_string(drone_id) + "/action/drone", true);
        ac.waitForServer();
        ac.sendGoal(goal);

        actionlib::SimpleClientGoalState::StateEnum state;
        std::vector<std::string> new_plan;
        bool has_new_plan = false;

        do {
          ros::Duration(0.01).sleep();
          state = ac.getState().state_;

          // Check if plan has changed
          if (g_kb.getPlanIfPlanHasChanged(executor_id, new_plan)
             && (new_plan.empty() || !std::equal(plan_it, plan.end(), new_plan.begin())))
          {
            has_new_plan = true;
            break;
          }
        } while (state == actionlib::SimpleClientGoalState::PENDING || state == actionlib::SimpleClientGoalState::ACTIVE);

        if (has_new_plan)
        {
          feedback.status = urs_wearable::Feedback::STATUS_REPLANNED;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": REPLANNED");

          ac.cancelGoal();
          ros_warn("p" + std::to_string(executor_id) + ": drone action finished with state " + ac.getState().toString());

          if (new_plan.empty())
          {
            if (g_kb.getUnsatisfiedGoals(req.goal).empty())
            {
              feedback.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
              g_feedback_pub.publish(feedback);
              ros_warn("p" + std::to_string(executor_id) + ": SUCCEEDED");
            }
            else
            {
              feedback.status = urs_wearable::Feedback::STATUS_PREEMPTED;
              g_feedback_pub.publish(feedback);
              ros_warn("p" + std::to_string(executor_id) + ": PREEMPTED");
            }

            g_kb.unregisterExecutor(executor_id);
            return;
          }

          plan = new_plan;
          plan_it = plan.begin();
          actions = g_kb.parsePlan(new_plan);
          actions_it = actions.begin();

          continue;
        }

        ros_warn("p" + std::to_string(executor_id) + ": drone action finished with state " + ac.getState().toString());

        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          feedback.status = urs_wearable::Feedback::STATUS_PREEMPTED;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": PREEMPTED");

          g_kb.unregisterExecutor(executor_id);
          return;
        }
      }
      else
      {
        // Don't update/insert the effects of the action if the plan has changed
        // This check is for those actions that don't require drone action
        std::vector<std::string> new_plan;
        if (g_kb.getPlanIfPlanHasChanged(executor_id, new_plan)
            && (new_plan.empty() || !std::equal(plan_it, plan.end(), new_plan.begin())))
        {
          feedback.status = urs_wearable::Feedback::STATUS_REPLANNED;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": REPLANNED");

          if (new_plan.empty())
          {
            if (g_kb.getUnsatisfiedGoals(req.goal).empty())
            {
              feedback.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
              g_feedback_pub.publish(feedback);
              ros_warn("p" + std::to_string(executor_id) + ": SUCCEEDED");
            }
            else
            {
              feedback.status = urs_wearable::Feedback::STATUS_PREEMPTED;
              g_feedback_pub.publish(feedback);
              ros_warn("p" + std::to_string(executor_id) + ": PREEMPTED");
            }
            g_kb.unregisterExecutor(executor_id);
            return;
          }

          plan = new_plan;
          plan_it = plan.begin();
          actions = g_kb.parsePlan(new_plan);
          actions_it = actions.begin();

          continue;
        }
      }

      // Update/Insert the effects of the action
      if (effects.size() > 0)
      {
        g_kb.upsertPredicates(effects);
      }

      ++plan_it;
      ++actions_it;
    }

    feedback.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
    g_feedback_pub.publish(feedback);
    ros_warn("p" + std::to_string(executor_id) + ": SUCCEEDED");
  }
  else if (g_kb.getUnsatisfiedGoals(req.goal).empty())
  {
    feedback.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
    g_feedback_pub.publish(feedback);
    ros_warn("p" + std::to_string(executor_id) + ": SUCCEEDED");
  }
  else
  {
    feedback.status = urs_wearable::Feedback::STATUS_REJECTED;
    g_feedback_pub.publish(feedback);
    ros_warn("p" + std::to_string(executor_id) + ": REJECTED");
  }

  g_kb.unregisterExecutor(executor_id);
}

bool getStateService(urs_wearable::GetState::Request& req, urs_wearable::GetState::Response& res)
{
  g_kb.publish();
  return true;
}

LocationTable::loc_id_t addLocation(const geometry_msgs::Pose& pose)
{
  LocationTable::loc_id_t loc_id = g_kb.loc_table_.insertLocation(pose);
  std::vector<urs_wearable::Predicate> aux_preds;
  urs_wearable::Predicate pred;
  pred.truth_value = true;

  // Generate auxiliary predicates
  auto loc_map_lt = g_kb.loc_table_.loc_map_.lock_table();
  for (const auto& loc : loc_map_lt)
  {
    if (loc.first != loc_id)
    {
      // above(l0,l1)
      pred.type = urs_wearable::Predicate::TYPE_ABOVE;
      if (loc.second.position.z > pose.position.z)
      {
        pred.above.l0.value = loc.first;
        pred.above.l1.value = loc_id;
      }
      else
      {
        pred.above.l0.value = loc_id;
        pred.above.l1.value = loc.first;
      }
      aux_preds.push_back(pred);

      // aligned(l0,l1)
      if (std::fabs(loc.second.position.z - pose.position.z) < 0.5)
      {
        pred.type = urs_wearable::Predicate::TYPE_ALIGNED;

        pred.aligned.l0.value = loc.first;
        pred.aligned.l1.value = loc_id;
        aux_preds.push_back(pred);

        pred.aligned.l0.value = loc_id;
        pred.aligned.l1.value = loc.first;
        aux_preds.push_back(pred);
      }

      // collided(l0,l1)
      if (pointDistance3D(loc.second.position, pose.position) < 2.0)
      {
        pred.type = urs_wearable::Predicate::TYPE_COLLIDED;

        pred.collided.l0.value = loc.first;
        pred.collided.l1.value = loc_id;
        aux_preds.push_back(pred);

        pred.collided.l0.value = loc_id;
        pred.collided.l1.value = loc.first;
        aux_preds.push_back(pred);
      }
    }
  }
  loc_map_lt.unlock();
  g_kb.upsertPredicates(aux_preds);

  return loc_id;
}

bool addAreaService(urs_wearable::AddArea::Request& req, urs_wearable::AddArea::Response& res)
{
  LocationTable::Area area;
  area.loc_id_left = req.loc_id_left;
  area.loc_id_right = req.loc_id_right;

  LocationTable::area_id_t area_id = g_kb.loc_table_.insertArea(area);
  std::vector<urs_wearable::Predicate> aux_preds;
  urs_wearable::Predicate pred;
  pred.truth_value = true;
  pred.type = urs_wearable::Predicate::TYPE_IN;

  // Generate auxiliary predicates
  auto loc_map_lt = g_kb.loc_table_.loc_map_.lock_table();
  for (const auto& loc : loc_map_lt)
  {
    // in(l,a)
    if (LocationTable::within_area(loc.second, area))
    {
      pred.in.l.value = loc.first;
      pred.in.a.value = area_id;

      aux_preds.push_back(pred);
    }
  }
  loc_map_lt.unlock();
  g_kb.upsertPredicates(aux_preds);

  res.area_id = area_id;

  return true;
}

bool addLocationService(urs_wearable::AddLocation::Request& req, urs_wearable::AddLocation::Response& res)
{
  res.loc_id = addLocation(req.pose);
  return true;
}

// TODO
bool removeAreaService(urs_wearable::RemoveArea::Request& req, urs_wearable::RemoveArea::Response& res)
{
  return true;
}

// TODO
bool removeLocationService(urs_wearable::RemoveLocation::Request& req, urs_wearable::RemoveLocation::Response& res)
{
  // Erase predicates associated with req.loc_id

  // Erase req.loc_id from the location table

  return true;
}


bool setGoalService(urs_wearable::SetGoal::Request& req, urs_wearable::SetGoal::Response& res)
{
  res.executor_id = g_kb.registerExecutor();
  std::thread thread_executor(execute, res.executor_id, req);
  thread_executor.detach();
  return true;
}

void scan(int uav_id, geometry_msgs::Point position_sw, geometry_msgs::Point position_ne)
{
  std::vector<geometry_msgs::Point> v;

  int i = 0;
  for (double y = position_sw.y; y < position_ne.y; y += 1.0, i++)
  {
    geometry_msgs::Point point;
    if (i % 2 == 0)
    {
      point.x = position_sw.x;
      point.y = y;
      point.z = 4;
      v.push_back(point);

      point.x = position_ne.x;
      point.y = y;
      point.z = 4;
      v.push_back(point);
    }
    else
    {
      point.x = position_ne.x;
      point.y = y;
      point.z = 4;
      v.push_back(point);

      point.x = position_sw.x;
      point.y = y;
      point.z = 4;
      v.push_back(point);
    }
  }

  urs_wearable::SetPosition set_position_srv;
  std::cout << "%%%%SIZE:::  " << v.size() << std::endl;
  for (i = 0; i < v.size(); i++)
  {
    set_position_srv.request.position = v[i];
    ros::service::call(g_uav_ns + std::to_string(uav_id) + "/set_position", set_position_srv);

    geometry_msgs::PoseStamped::ConstPtr pose_stamped;
    do
    {
      pose_stamped = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(g_uav_ns + std::to_string(uav_id) + "/ground_truth_to_tf/pose");
    } while (pointDistance2D(pose_stamped->pose.position, v[i]) > 0.1);
  }
}

//bool scanService(urs_wearable::Scan::Request& req, urs_wearable::Scan::Response& res)
//{
//  std::thread t(scan, req.uav_id, req.position_sw, req.position_ne);
//  t.detach();
//  return true;
//}
//
//bool gatherService(urs_wearable::Gather::Request& req, urs_wearable::Gather::Response& res)
//{
//  for (int i = 0; i < req.uav_id.size(); i++)
//  {
//    geometry_msgs::Point target_position;
//    target_position = req.position;
//
//    switch (req.uav_id[i])
//    {
//      case 0:
//        target_position.x += 2;
//        break;
//      case 1:
//        target_position.x -= 2;
//        break;
//      case 2:
//        target_position.y += 2;
//        break;
//      case 3:
//        target_position.y -= 2;
//        break;
//    }
//
//    urs_wearable::SetPosition set_position_srv;
//    set_position_srv.request.position = target_position;
//    ros::service::call(g_uav_ns + std::to_string(req.uav_id[i]) + "/set_position", set_position_srv);
//  }
//  return true;
//}

void battery(const std_msgs::StringConstPtr& s)
{
  std::vector<std::string> tokens = tokenizeString(s->data, ":");
  try
  {
    std::vector<std::string> drone_id_tokens = tokenizeString(tokens[1], " ");
    int drone_id = std::stoi(drone_id_tokens.back());
    int battery_value = (tokens.size() == 3)? std::stoi(trim(tokens[2])): std::stoi(trim(tokens[3]));

    // Update the KB and land the drone if its battery value is less than a specified value
    if (battery_value < 10)
    {
      urs_wearable::Predicate pred;
      pred.type = urs_wearable::Predicate::TYPE_LOW_BATTERY;
      pred.truth_value = true;
      pred.low_battery.d.value = drone_id;
      g_kb.upsertPredicates(std::vector<urs_wearable::Predicate>{pred});

      urs_wearable::DroneGoal goal;
      goal.action_type = urs_wearable::DroneGoal::TYPE_LAND;
      actionlib::SimpleActionClient<urs_wearable::DroneAction> ac("/uav" + std::to_string(drone_id) + "/action/drone", true);
      ac.waitForServer();
      ac.sendGoal(goal);

      //TODO: Update at predicate after landing?
    }
  }
  catch(std::exception& e)
  {
    ros_error("Error extracting battery level message: " + s->data);
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "exec_monitor");
  ros::NodeHandle nh;

  int uav_total;
  retrieve("~uav_total", uav_total);
  retrieve("~uav_ns", g_uav_ns);

  // Set KB connection with a planner
  retrieve("~domain_file", g_kb.domain_file);
  retrieve("~planner_command", g_kb.planner_command);
  retrieve("~problem_path", g_kb.problem_path);
  retrieve("~tmp_path", g_kb.tmp_path);

  // Set KB state publisher
  ros::Publisher state_pub = nh.advertise<urs_wearable::State>("urs_wearable/state", 10, true);
  g_kb.setStatePub(&state_pub);

  // Set feedback publisher
  g_feedback_pub = nh.advertise<urs_wearable::Feedback>("urs_wearable/feedback", 10, true);

  // Set initial state
  std::vector<urs_wearable::Predicate> initial_state;

  urs_wearable::Predicate pred;
  pred.type = urs_wearable::Predicate::TYPE_AT;
  pred.truth_value = true;

  for (int i = 0; i < uav_total; i++)
  {
    geometry_msgs::PoseStamped::ConstPtr pose_stamped =
        ros::topic::waitForMessage<geometry_msgs::PoseStamped>(g_uav_ns + std::to_string(i) + "/ground_truth_to_tf/pose");

    pred.at.d.value = i;
    pred.at.l.value = addLocation(pose_stamped->pose);
    initial_state.push_back(pred);
  }
  g_kb.upsertPredicates(initial_state);

  // Subscribe to topics
  ros::Subscriber battery_subscribe = nh.subscribe("/w_battery_value", 1, battery);

  // Advertise services
  ros::ServiceServer add_area_service = nh.advertiseService("urs_wearable/add_area", addAreaService);
  ros::ServiceServer add_location_service = nh.advertiseService("urs_wearable/add_location", addLocationService);
  ros::ServiceServer get_state_service = nh.advertiseService("urs_wearable/get_state", getStateService);
  ros::ServiceServer remove_area_service = nh.advertiseService("urs_wearable/remove_area",removeAreaService);
  ros::ServiceServer remove_location_service = nh.advertiseService("urs_wearable/remove_location", removeLocationService);
  ros::ServiceServer set_goal_service = nh.advertiseService("urs_wearable/set_goal", setGoalService);

  ros_info("Waiting for connections from wearable devices...");

  ros::spin();

  return EXIT_SUCCESS;
}
