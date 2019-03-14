#include <string>
#include <thread>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <libcuckoo/cuckoohash_map.hh>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <urs_wearable/Action.h>
#include <urs_wearable/AddArea.h>
#include <urs_wearable/AddDroneGoal.h>
#include <urs_wearable/AddLocation.h>
#include <urs_wearable/DroneAction.h>
#include <urs_wearable/Feedback.h>
#include <urs_wearable/GetState.h>
#include <urs_wearable/RemoveArea.h>
#include <urs_wearable/RemoveLocation.h>
#include <urs_wearable/SetDroneActionResult.h>
#include <urs_wearable/SetGoal.h>
#include <urs_wearable/SetPosition.h>

#include "urs_wearable/common.h"
#include "urs_wearable/knowledge_base.h"

// We use uint8_t here to match with the type of 'value' in ObjectDroneID.msg
typedef std::uint8_t drone_id_type;

KnowledgeBase g_kb("urs", "urs_problem");
ros::Publisher g_feedback_pub;
std::string g_uav_ns;

std::vector<int> g_battery_level;
std::vector<bool> g_emergency_landed;

void getAreaBorders(const geometry_msgs::Pose& pose_left, const geometry_msgs::Pose& pose_right,
                    geometry_msgs::Pose& pose_nw, geometry_msgs::Pose& pose_ne, geometry_msgs::Pose& pose_sw, geometry_msgs::Pose& pose_se)
{
  if (pose_left.position.y > pose_right.position.y)
  {
    pose_nw.position.x = pose_left.position.x;
    pose_nw.position.y = pose_right.position.y;

    pose_ne.position.x = pose_right.position.x;
    pose_ne.position.y = pose_right.position.y;

    pose_sw.position.x = pose_left.position.x;
    pose_sw.position.y = pose_left.position.y;

    pose_se.position.x = pose_right.position.x;
    pose_se.position.y = pose_left.position.y;
  }
  else
  {
    pose_nw.position.x = pose_left.position.x;
    pose_nw.position.y = pose_left.position.y;

    pose_ne.position.x = pose_right.position.x;
    pose_ne.position.y = pose_left.position.y;

    pose_sw.position.x = pose_left.position.x;
    pose_sw.position.y = pose_right.position.y;

    pose_se.position.x = pose_right.position.x;
    pose_se.position.y = pose_right.position.y;
  }

  pose_nw.position.z
  = pose_ne.position.z
  = pose_sw.position.z
  = pose_se.position.z
  = (pose_left.position.z > pose_right.position.z) ? pose_left.position.z : pose_right.position.z;
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

      // alignedpush_back(l0,l1)
      if (pointDistance2D(loc.second.position, pose.position) < 0.5)
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

  pred.type = urs_wearable::Predicate::TYPE_IN;
  auto area_map_lt = g_kb.loc_table_.area_map_.lock_table();
  for (const auto& area : area_map_lt)
  {
    geometry_msgs::Pose pose_left, pose_right, pose_nw, pose_ne, pose_sw, pose_se;
    g_kb.loc_table_.loc_map_.find(area.second.loc_id_left, pose_left);
    g_kb.loc_table_.loc_map_.find(area.second.loc_id_right, pose_right);
    getAreaBorders(pose_left, pose_right, pose_nw, pose_ne, pose_sw, pose_se);

    // in(l,a)
    if (pose.position.x >= pose_nw.position.x
        && pose.position.x <= pose_ne.position.x
        && pose.position.y >= pose_sw.position.y
        && pose.position.y <= pose_nw.position.y
        && pose.position.z >= pose_nw.position.z - 0.5
        && pose.position.z <= pose_nw.position.z + 0.5)
    {
      pred.in.l.value = loc_id;
      pred.in.a.value = area.first;

      aux_preds.push_back(pred);
    }
  }
  area_map_lt.unlock();

  g_kb.upsertPredicates(aux_preds);

  return loc_id;
}

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

//          require_drone_action = true;
          drone_id = actions_it->ascend.d.value;
//          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
//          goal.poses.push_back(pose_to);

          ///////////////////////////////////////////////////////////////////////////////////////////////////////
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.poses.push_back(pose_to);

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

//          require_drone_action = true;
          drone_id = actions_it->descend.d.value;
//          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
//          goal.poses.push_back(pose_to);

          ///////////////////////////////////////////////////////////////////////////////////////////////////////
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.poses.push_back(pose_to);

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

        case urs_wearable::Action::TYPE_GATHER:
        {
          geometry_msgs::PoseStamped::ConstPtr pose_stamped =
              ros::topic::waitForMessage<geometry_msgs::PoseStamped>(g_uav_ns + std::to_string(actions_it->scan.d.value) + "/ground_truth_to_tf/pose");

          geometry_msgs::Pose pose_to;
          g_kb.loc_table_.loc_map_.find(actions_it->gather.l1.value, pose_to);

          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->gather.NAME + "(" + std::to_string(actions_it->gather.d.value) + ",("
                   + std::to_string(pose_to.position.x) + "," + std::to_string(pose_to.position.y) + "," + std::to_string(pose_to.position.z) + "))");

//          require_drone_action = true;
          drone_id = actions_it->gather.d.value;
//          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          pose_to.position.z += drone_id + 1.0;

          geometry_msgs::Pose pose_tmp = pose_stamped->pose;
          pose_tmp.position.z = pose_to.position.z;

//          goal.poses.push_back(pose_tmp);   // elevate the drone to help avoiding collision
//          goal.poses.push_back(pose_to);    // then move in 2D

          ///////////////////////////////////////////////////////////////////////////////////////////////////////
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.poses.push_back(pose_tmp);
          drone_goal.poses.push_back(pose_to);

          urs_wearable::AddDroneGoal add_drone_goal_srv;
          add_drone_goal_srv.request.drone_goals.push_back(drone_goal);
          ros::service::call("uav" + std::to_string(drone_id) + "/add_drone_goal", add_drone_goal_srv);

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_AT;
          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->gather.l0.value;
          effect.truth_value = false;
          effects.push_back(effect);

          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->gather.l1.value;
          effect.truth_value = true;
          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_LAND:
        {
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->land.NAME + "(" + std::to_string(actions_it->land.d.value) + ")");

//          require_drone_action = true;
          drone_id = actions_it->land.d.value;
//          goal.action_type = urs_wearable::DroneGoal::TYPE_LAND;

          ///////////////////////////////////////////////////////////////////////////////////////////////////////
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_LAND;
          drone_goal.executor_id = executor_id;

          urs_wearable::AddDroneGoal add_drone_goal_srv;
          add_drone_goal_srv.request.drone_goals.push_back(drone_goal);
          ros::service::call("uav" + std::to_string(drone_id) + "/add_drone_goal", add_drone_goal_srv);

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

//          require_drone_action = true;
          drone_id = actions_it->move.d.value;
//          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
//          goal.poses.push_back(pose_to);

          ///////////////////////////////////////////////////////////////////////////////////////////////////////
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.poses.push_back(pose_to);

          urs_wearable::AddDroneGoal add_drone_goal_srv;
          add_drone_goal_srv.request.drone_goals.push_back(drone_goal);
          ros::service::call("uav" + std::to_string(drone_id) + "/add_drone_goal", add_drone_goal_srv);

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

        case urs_wearable::Action::TYPE_SCAN:
        {
          geometry_msgs::PoseStamped::ConstPtr pose_stamped =
              ros::topic::waitForMessage<geometry_msgs::PoseStamped>(g_uav_ns + std::to_string(actions_it->scan.d.value) + "/ground_truth_to_tf/pose");

          LocationTable::Area area_to;
          g_kb.loc_table_.area_map_.find(actions_it->scan.a.value, area_to);

          geometry_msgs::Pose pose_left, pose_right;
          g_kb.loc_table_.loc_map_.find(area_to.loc_id_left, pose_left);
          g_kb.loc_table_.loc_map_.find(area_to.loc_id_right, pose_right);

          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->scan.NAME + "(" + std::to_string(actions_it->scan.d.value) + ",(("
                   + std::to_string(pose_left.position.x) + "," + std::to_string(pose_left.position.y) + "," + std::to_string(pose_left.position.z) + "),("
                   + std::to_string(pose_right.position.x) + "," + std::to_string(pose_right.position.y) + "," + std::to_string(pose_right.position.z) + ")))");

          geometry_msgs::Pose pose_nw, pose_ne, pose_sw, pose_se;
          getAreaBorders(pose_left, pose_right, pose_nw, pose_ne, pose_sw, pose_se);
          double dist[4];
          dist[0] = pointDistance2D(pose_stamped->pose.position, pose_nw.position);
          dist[1] = pointDistance2D(pose_stamped->pose.position, pose_ne.position);
          dist[2] = pointDistance2D(pose_stamped->pose.position, pose_sw.position);
          dist[3] = pointDistance2D(pose_stamped->pose.position, pose_se.position);

          double min_dist = std::numeric_limits<double>::max();
          int min_dist_index = 0;
          for (int i = 0; i < 4; i++)
          {
            if (dist[0] < min_dist)
            {
              min_dist = dist[0];
              min_dist_index = i;
            }
          }

//          require_drone_action = true;
          drone_id = actions_it->scan.d.value;
//          goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
//          switch(min_dist_index)
//          {
//            case 0:
//              goal.poses.push_back(pose_nw);
//              goal.poses.push_back(pose_ne);
//              goal.poses.push_back(pose_se);
//              goal.poses.push_back(pose_sw);
//              goal.poses.push_back(pose_nw);
//              break;
//
//            case 1:
//              goal.poses.push_back(pose_ne);
//              goal.poses.push_back(pose_se);
//              goal.poses.push_back(pose_sw);
//              goal.poses.push_back(pose_nw);
//              goal.poses.push_back(pose_ne);
//              break;
//
//            case 2:
//              goal.poses.push_back(pose_sw);
//              goal.poses.push_back(pose_nw);
//              goal.poses.push_back(pose_ne);
//              goal.poses.push_back(pose_se);
//              goal.poses.push_back(pose_sw);
//              break;
//
//            case 3:
//              goal.poses.push_back(pose_se);
//              goal.poses.push_back(pose_sw);
//              goal.poses.push_back(pose_nw);
//              goal.poses.push_back(pose_ne);
//              goal.poses.push_back(pose_se);
//              break;
//          }

          ///////////////////////////////////////////////////////////////////////////////////////////////////////
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;

          switch(min_dist_index)
          {
            case 0:
              drone_goal.poses.push_back(pose_nw);
              drone_goal.poses.push_back(pose_ne);
              drone_goal.poses.push_back(pose_se);
              drone_goal.poses.push_back(pose_sw);
              drone_goal.poses.push_back(pose_nw);
              break;

            case 1:
              drone_goal.poses.push_back(pose_ne);
              drone_goal.poses.push_back(pose_se);
              drone_goal.poses.push_back(pose_sw);
              drone_goal.poses.push_back(pose_nw);
              drone_goal.poses.push_back(pose_ne);
              break;

            case 2:
              drone_goal.poses.push_back(pose_sw);
              drone_goal.poses.push_back(pose_nw);
              drone_goal.poses.push_back(pose_ne);
              drone_goal.poses.push_back(pose_se);
              drone_goal.poses.push_back(pose_sw);
              break;

            case 3:
              drone_goal.poses.push_back(pose_se);
              drone_goal.poses.push_back(pose_sw);
              drone_goal.poses.push_back(pose_nw);
              drone_goal.poses.push_back(pose_ne);
              drone_goal.poses.push_back(pose_se);
              break;
          }

          urs_wearable::AddDroneGoal add_drone_goal_srv;
          add_drone_goal_srv.request.drone_goals.push_back(drone_goal);
          ros::service::call("uav" + std::to_string(drone_id) + "/add_drone_goal", add_drone_goal_srv);

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_AT;
          effect.at.d.value = drone_id;
          effect.at.l.value = actions_it->scan.l.value;
          effect.truth_value = false;
          effects.push_back(effect);

          effect.at.d.value = drone_id;
          effect.at.l.value = addLocation(pose_nw);
          effect.truth_value = true;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_SCANNED;
          effect.scanned.d.value = drone_id;
          effect.scanned.a.value = actions_it->scan.a.value;
          effect.truth_value = true;
          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_TAKEOFF:
        {
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          g_feedback_pub.publish(feedback);
          ros_warn("p" + std::to_string(executor_id) + ": ACTIVE "
                   + actions_it->takeoff.NAME + "(" + std::to_string(actions_it->takeoff.d.value) + ")");

//          require_drone_action = true;
          drone_id = actions_it->takeoff.d.value;
//          goal.action_type = urs_wearable::DroneGoal::TYPE_TAKEOFF;

          ///////////////////////////////////////////////////////////////////////////////////////////////////////
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_TAKEOFF;
          drone_goal.executor_id = executor_id;

          urs_wearable::AddDroneGoal add_drone_goal_srv;
          add_drone_goal_srv.request.drone_goals.push_back(drone_goal);
          ros::service::call("uav" + std::to_string(drone_id) + "/add_drone_goal", add_drone_goal_srv);

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_HOVERED;
          effect.hovered.d.value = drone_id;
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

  geometry_msgs::Pose pose_left, pose_right, pose_nw, pose_ne, pose_sw, pose_se;
  g_kb.loc_table_.loc_map_.find(req.loc_id_left, pose_left);
  g_kb.loc_table_.loc_map_.find(req.loc_id_right, pose_right);
  getAreaBorders(pose_left, pose_right, pose_nw, pose_ne, pose_sw, pose_se);

  // Generate auxiliary predicates
  auto loc_map_lt = g_kb.loc_table_.loc_map_.lock_table();
  for (const auto& loc : loc_map_lt)
  {
    // in(l,a)
    if (loc.second.position.x >= pose_nw.position.x
        && loc.second.position.x <= pose_ne.position.x
        && loc.second.position.y >= pose_sw.position.y
        && loc.second.position.y <= pose_nw.position.y
        && loc.second.position.z >= pose_nw.position.z - 0.5
        && loc.second.position.z <= pose_nw.position.z + 0.5)
    {
      pred.in.l.value = loc.first;
      pred.in.a.value = area_id;

      aux_preds.push_back(pred);

      ros_error("Predicated in/2 added");
    }
  }
  loc_map_lt.unlock();
  g_kb.upsertPredicates(aux_preds);

  res.area_id = area_id;

  // Log the request
  ROS_INFO_STREAM("Received an add_area request" << std::endl
                  << "loc_id_left: " << std::to_string(req.loc_id_left) << std::endl
                  << "loc_id_right: " << std::to_string(req.loc_id_right) << std::endl
                  << "(returned) area_id: " << std::to_string(res.area_id));
  return true;
}

bool addLocationService(urs_wearable::AddLocation::Request& req, urs_wearable::AddLocation::Response& res)
{
  res.loc_id = addLocation(req.pose);

  // Log the request
  ROS_INFO_STREAM("Received an add_location request" << std::endl
                  << req.pose << std::endl
                  << "(returned) loc_id: " << std::to_string(res.loc_id));
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
  std::thread executor_thread(execute, res.executor_id, req);
  executor_thread.detach();

  // Log the request
  ROS_INFO_STREAM("Received a set_goal request" << std::endl
                  << "player_id: " << std::to_string(req.player_id) << std::endl
                  << "goal: " << KnowledgeBase::getPredicateString(req.goal) << std::endl
                  << "(returned) executor_id: " << std::to_string(res.executor_id));
  return true;
}

void land(drone_id_type drone_id)
{
  urs_wearable::Predicate pred;
  pred.type = urs_wearable::Predicate::TYPE_LOW_BATTERY;
  pred.truth_value = true;
  pred.low_battery.d.value = drone_id;
  g_kb.upsertPredicates(std::vector<urs_wearable::Predicate>{pred});

  // FIXME (need to be changed after updating drone_action)
  urs_wearable::DroneGoal goal;
  goal.action_type = urs_wearable::DroneGoal::TYPE_LAND;
  actionlib::SimpleActionClient<urs_wearable::DroneAction> ac("/uav" + std::to_string(drone_id) + "/action/drone", true);
  ac.waitForServer();
  ac.sendGoal(goal);

  if (ac.waitForResult() && ac.getResult()->success)
  {
    pred.type = urs_wearable::Predicate::TYPE_HOVERED;
    pred.truth_value = false;
    pred.hovered.d.value = drone_id;
    g_kb.upsertPredicates(std::vector<urs_wearable::Predicate>{pred});
  }
}

void battery(const std_msgs::StringConstPtr& s)
{
  std::vector<std::string> tokens = tokenizeString(s->data, " {':,}");
  try
  {
    drone_id_type drone_id = -1;
    int battery_value = -1;
    for (size_t i = 0; i < tokens.size(); i++)
    {
      if (tokens[i] == "drone_id")
      {
//        ros_warn("drone_id index: " + std::to_string(i));
        drone_id = std::stoi(tokens[i + 1]);
      }
      else if (tokens[i] == "battery_value")
      {
//        ros_warn("battery_value index: " + std::to_string(i));
        battery_value = std::stoi(tokens[i + 1]);
      }
    }

//    drone_id_type drone_id = std::stoi(tokens[11]);
//    int battery_value = std::stoi(tokens[13]);

    // Update the KB and land the drone if its battery value is less than a specified value
    if (battery_value < 10 && !g_emergency_landed[drone_id])
    {
      g_emergency_landed[drone_id] = true;

      std::thread land_thread(land, drone_id);
      land_thread.detach();
    }

    if (g_battery_level[drone_id] != battery_value)
    {
      g_battery_level[drone_id] = battery_value;

      // Log the change of battery level
      ROS_INFO_STREAM("Battery" << std::endl
                      << "drone_id: " << std::to_string(drone_id) << std::endl
                      << "battery: " << std::to_string(battery_value));
    }
  }
  catch(std::exception& e)
  {
    ros_error("Error extracting battery level message: " + s->data);
  }
}

bool setDroneActionResultService(urs_wearable::SetDroneActionResult::Request& req, urs_wearable::SetDroneActionResult::Response& res)
{


  return true;
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

    // Set battery level vector to be used for logging purpose
    g_battery_level.push_back(-1);
    g_emergency_landed.push_back(false);
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
  ros::ServiceServer set_drone_action_result_service = nh.advertiseService("urs_wearable/set_drone_action_result", setDroneActionResultService);
  ros::ServiceServer set_goal_service = nh.advertiseService("urs_wearable/set_goal", setGoalService);

  ros_info("Waiting for connections from wearable devices...");

  ros::spin();

  return EXIT_SUCCESS;
}
