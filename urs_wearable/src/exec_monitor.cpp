#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <libcuckoo/cuckoohash_map.hh>
#include <ros/ros.h>

#include "urs_wearable/common.h"
#include "urs_wearable/knowledge_base.h"

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <urs_wearable/Action.h>
#include <urs_wearable/AddArea.h>
#include <urs_wearable/AddDroneGoal.h>
#include <urs_wearable/AddLocation.h>
#include <urs_wearable/DroneAction.h>
#include <urs_wearable/Feedback.h>
#include <urs_wearable/GetState.h>
#include <urs_wearable/RemoveArea.h>
#include <urs_wearable/RemoveDroneGoal.h>
#include <urs_wearable/RemoveLocation.h>
#include <urs_wearable/SetDroneActionStatus.h>
#include <urs_wearable/SetGoal.h>
#include <urs_wearable/SetPosition.h>

// We use uint8_t here to match with the type of 'value' in ObjectDroneID.msg
typedef std::uint8_t drone_id_type;

KnowledgeBase g_kb("urs", "urs_problem");
ros::Publisher g_feedback_pub;
std::mutex g_feedback_pub_mutex_;
std::string g_uav_ns;
int g_uav_total;

std::vector<std::pair<int, bool>> g_drone_battery;

int g_unity_drone_id_index = -1;
int g_unity_battery_value_index = -1;

void getAreaBorders(const geometry_msgs::Pose& pose_left, const geometry_msgs::Pose& pose_right,
                    geometry_msgs::Pose& pose_nw, geometry_msgs::Pose& pose_ne, geometry_msgs::Pose& pose_sw, geometry_msgs::Pose& pose_se)
{
  if (pose_left.position.y > pose_right.position.y)
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
  else
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
    if (pose.position.x >= pose_sw.position.x
        && pose.position.x <= pose_ne.position.x
        && pose.position.y >= pose_sw.position.y
        && pose.position.y <= pose_ne.position.y
        && pose.position.z >= pose_sw.position.z - 0.5
        && pose.position.z <= pose_sw.position.z + 0.5)
    {
      pred.in.l.value = loc_id;
      pred.in.a.value = area.first;

      aux_preds.push_back(pred);
    }
  }
  area_map_lt.unlock();

  g_kb.upsertPredicates(0, aux_preds);

  return loc_id;
}

void execute(KnowledgeBase::executor_id_type executor_id)
{
  g_kb.executor_map_.find_fn(executor_id, [executor_id](const struct Executor& executor)
  {
    std::vector<std::vector<urs_wearable::DroneGoal>> drone_goals;
    drone_goals.reserve(g_uav_total);
    for (drone_id_type drone_id = 0; drone_id < g_uav_total; drone_id++)
    {
      drone_goals.push_back(std::vector<urs_wearable::DroneGoal>());
    }

    for (std::size_t action_index = 0; action_index < executor.actions.size(); action_index++)
    {
      const urs_wearable::Action& action = executor.actions[action_index];
      switch (action.type)
      {
        case urs_wearable::Action::TYPE_ASCEND:
        {
          geometry_msgs::Pose pose_to;
          g_kb.loc_table_.loc_map_.find(action.ascend.l1.value, pose_to);

          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.action_index = action_index;
          drone_goal.poses.push_back(pose_to);

          drone_goals[action.ascend.d.value].push_back(drone_goal);
        }
        break;

        case urs_wearable::Action::TYPE_DESCEND:
        {
          geometry_msgs::Pose pose_to;
          g_kb.loc_table_.loc_map_.find(action.descend.l1.value, pose_to);

          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.action_index = action_index;
          drone_goal.poses.push_back(pose_to);

          drone_goals[action.descend.d.value].push_back(drone_goal);
        }
        break;

        case urs_wearable::Action::TYPE_GATHER:
        {
          geometry_msgs::PoseStamped::ConstPtr pose_stamped =
              ros::topic::waitForMessage<geometry_msgs::PoseStamped>(g_uav_ns + std::to_string(action.scan.d.value) + "/ground_truth_to_tf/pose");

          geometry_msgs::Pose pose_to;
          g_kb.loc_table_.loc_map_.find(action.gather.l1.value, pose_to);

          geometry_msgs::Pose pose_tmp = pose_stamped->pose;
          pose_tmp.position.z = pose_to.position.z;

          drone_id_type drone_id = action.gather.d.value;
          pose_to.position.z += drone_id + 1.0;

          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.action_index = action_index;
          drone_goal.poses.push_back(pose_tmp);
          drone_goal.poses.push_back(pose_to);

          drone_goals[drone_id].push_back(drone_goal);
        }
        break;

        case urs_wearable::Action::TYPE_LAND:
        {
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_LAND;
          drone_goal.executor_id = executor_id;
          drone_goal.action_index = action_index;

          drone_goals[action.land.d.value].push_back(drone_goal);
        }
        break;

        case urs_wearable::Action::TYPE_MOVE:
        {
          geometry_msgs::Pose pose_to;
          g_kb.loc_table_.loc_map_.find(action.move.l1.value, pose_to);

          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.action_index = action_index;
          drone_goal.poses.push_back(pose_to);

          drone_goals[action.move.d.value].push_back(drone_goal);
        }
        break;

        case urs_wearable::Action::TYPE_SCAN:
        {
          geometry_msgs::PoseStamped::ConstPtr pose_stamped =
              ros::topic::waitForMessage<geometry_msgs::PoseStamped>(g_uav_ns + std::to_string(action.scan.d.value) + "/ground_truth_to_tf/pose");

          LocationTable::Area area_to;
          g_kb.loc_table_.area_map_.find(action.scan.a.value, area_to);

          geometry_msgs::Pose pose_left, pose_right;
          g_kb.loc_table_.loc_map_.find(area_to.loc_id_left, pose_left);
          g_kb.loc_table_.loc_map_.find(area_to.loc_id_right, pose_right);

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
            if (dist[i] < min_dist)
            {
              min_dist = dist[i];
              min_dist_index = i;
            }
          }

          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_MOVE;
          drone_goal.executor_id = executor_id;
          drone_goal.action_index = action_index;

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

          drone_goals[action.scan.d.value].push_back(drone_goal);
        }
        break;

        case urs_wearable::Action::TYPE_TAKEOFF:
        {
          urs_wearable::DroneGoal drone_goal;
          drone_goal.action_type = urs_wearable::DroneGoal::TYPE_TAKEOFF;
          drone_goal.executor_id = executor_id;
          drone_goal.action_index = action_index;

          drone_goals[action.takeoff.d.value].push_back(drone_goal);
        }
        break;

        default:
        {
          urs_wearable::Feedback feedback;
          feedback.executor_id = executor_id;
          feedback.status = urs_wearable::Feedback::STATUS_ABORTED;
          feedback.message = "p" + std::to_string(executor_id) + ": Unrecognized action type";
          {
            std::lock_guard<std::mutex> lock(g_feedback_pub_mutex_);
            g_feedback_pub.publish(feedback);
          }
          ros_error("p" + std::to_string(executor_id) + ": Unrecognized action type");

          g_kb.unregisterExecutor(executor_id);
          return;
        }
      }
    }

    for (drone_id_type drone_id = 0; drone_id < g_uav_total; drone_id++)
    {
      if (!drone_goals[drone_id].empty())
      {
        urs_wearable::AddDroneGoal add_drone_goal_srv;
        add_drone_goal_srv.request.drone_goals = drone_goals[drone_id];
        if (!ros::service::call("/uav" + std::to_string(drone_id) + "/add_drone_goal", add_drone_goal_srv))
        {
          ros_error("p" + std::to_string(executor_id) + ": Cannot call service "
                    + ros::names::resolve("/uav" + std::to_string(drone_id) + "/add_drone_goal"));
        }
      }
    }
  });
}

void planAndExecute(KnowledgeBase::executor_id_type executor_id, const std::vector<urs_wearable::Predicate> goals)
{
  ros_warn("p" + std::to_string(executor_id) + ": PENDING");
  urs_wearable::Feedback feedback;
  feedback.executor_id = executor_id;
  feedback.status = urs_wearable::Feedback::STATUS_PENDING;
  {
    std::lock_guard<std::mutex> lock(g_feedback_pub_mutex_);
    g_feedback_pub.publish(feedback);
  }

  std::vector<std::string> plan;
  g_kb.plan(executor_id, goals, plan);

  if (plan.empty())
  {
    g_kb.unregisterExecutor(executor_id);

    ros_warn("p" + std::to_string(executor_id) + ": REJECTED");
    feedback.status = urs_wearable::Feedback::STATUS_REJECTED;
    std::lock_guard<std::mutex> lock(g_feedback_pub_mutex_);
    g_feedback_pub.publish(feedback);
  }
  else
  {
    std::string s = "p" + std::to_string(executor_id) + ": Plan = ";
    for (const auto& action : plan)
    {
      s += action + " ";
    }
    s.pop_back();
    ros_warn(s);

    execute(executor_id);
  }
}

bool setDroneActionStatusService(urs_wearable::SetDroneActionStatus::Request& req, urs_wearable::SetDroneActionStatus::Response& res)
{
  urs_wearable::Feedback feedback;
  feedback.executor_id = req.executor_id;
  feedback.status = req.status;
  feedback.message = req.message;

  bool done = false;
  switch (req.status)
  {
    case urs_wearable::Feedback::STATUS_ACTIVE:
      g_kb.executor_map_.find_fn(req.executor_id, [&feedback, &req](const struct Executor& executor)
      {
        const urs_wearable::Action& action = executor.actions[req.action_index];
        switch (action.type)
        {
          case urs_wearable::Action::TYPE_ASCEND:
            ros_warn("p" + std::to_string(req.executor_id) + ": ACTIVE "
                     + action.ascend.NAME + "("
                     + std::to_string(action.ascend.d.value) + ","
                     + std::to_string(action.ascend.l0.value) + ","
                     + std::to_string(action.ascend.l1.value) + ")");
            break;

          case urs_wearable::Action::TYPE_DESCEND:
            ros_warn("p" + std::to_string(req.executor_id) + ": ACTIVE "
                     + action.descend.NAME + "("
                     + std::to_string(action.descend.d.value) + ","
                     + std::to_string(action.descend.l0.value) + ","
                     + std::to_string(action.descend.l1.value) + ")");
            break;

          case urs_wearable::Action::TYPE_GATHER:
            ros_warn("p" + std::to_string(req.executor_id) + ": ACTIVE "
                     + action.gather.NAME + "("
                     + std::to_string(action.gather.d.value) + ","
                     + std::to_string(action.gather.l0.value) + ","
                     + std::to_string(action.gather.l1.value) + ")");
            break;

          case urs_wearable::Action::TYPE_LAND:
            ros_warn("p" + std::to_string(req.executor_id) + ": ACTIVE "
                     + action.land.NAME + "("
                     + std::to_string(action.land.d.value) + ")");
            break;

          case urs_wearable::Action::TYPE_MOVE:
            ros_warn("p" + std::to_string(req.executor_id) + ": ACTIVE "
                     + action.move.NAME + "("
                     + std::to_string(action.move.d.value) + ","
                     + std::to_string(action.move.l0.value) + ","
                     + std::to_string(action.move.l1.value) + ")");
            break;

          case urs_wearable::Action::TYPE_SCAN:
            ros_warn("p" + std::to_string(req.executor_id) + ": ACTIVE "
                     + action.scan.NAME + "("
                     + std::to_string(action.scan.d.value) + ","
                     + std::to_string(action.scan.l.value) + ","
                     + std::to_string(action.scan.a.value) + ")");
            break;

          case urs_wearable::Action::TYPE_TAKEOFF:
            ros_warn("p" + std::to_string(req.executor_id) + ": ACTIVE "
                     + action.takeoff.NAME + "("
                     + std::to_string(action.takeoff.d.value) + ")");
            break;

          default:
            ros_error("p" + std::to_string(req.executor_id) + ": Unrecognized action type");
        }

        feedback.current_action = action;
        std::lock_guard<std::mutex> lock(g_feedback_pub_mutex_);
        g_feedback_pub.publish(feedback);
      });
      break;

    case urs_wearable::Feedback::STATUS_REPLANNED:
      {
        urs_wearable::RemoveDroneGoal remove_drone_goal_srv;
        remove_drone_goal_srv.request.executor_id = req.executor_id;

        for (drone_id_type drone_id = 0; drone_id < g_uav_total; drone_id++)
        {
          if (!ros::service::call("/uav" + std::to_string(drone_id) + "/remove_drone_goal", remove_drone_goal_srv))
          {
            ros_error("p" + std::to_string(req.executor_id) + ": Cannot call service "
                      + ros::names::resolve("/uav" + std::to_string(drone_id) + "/remove_drone_goal"));
          }
        }

        std::thread execute_thread(execute, req.executor_id);
        execute_thread.detach();

        ros_warn("p" + std::to_string(req.executor_id) + ": REPLANNED");
        std::lock_guard<std::mutex> lock(g_feedback_pub_mutex_);
        g_feedback_pub.publish(feedback);
      }
      break;

    case urs_wearable::Feedback::STATUS_PREEMPTED:
      g_kb.executor_map_.find_fn(req.executor_id, [&feedback, &req, &done](const struct Executor& executor)
      {
        urs_wearable::RemoveDroneGoal remove_drone_goal_srv;
        remove_drone_goal_srv.request.executor_id = req.executor_id;

        for (drone_id_type drone_id = 0; drone_id < g_uav_total; drone_id++)
        {
          if (!ros::service::call("/uav" + std::to_string(drone_id) + "/remove_drone_goal", remove_drone_goal_srv))
          {
            ros_error("p" + std::to_string(req.executor_id) + ": Cannot call service "
                      + ros::names::resolve("/uav" + std::to_string(drone_id) + "/remove_drone_goal"));
          }
        }

        done = true;

        ros_warn("p" + std::to_string(req.executor_id) + ": PREEMPTED");
        std::lock_guard<std::mutex> lock(g_feedback_pub_mutex_);
        g_feedback_pub.publish(feedback);
      });
      break;

    case urs_wearable::Feedback::STATUS_ABORTED:
      g_kb.executor_map_.find_fn(req.executor_id, [&feedback, &req, &done](const struct Executor& executor)
      {
        urs_wearable::RemoveDroneGoal remove_drone_goal_srv;
        remove_drone_goal_srv.request.executor_id = req.executor_id;

        for (drone_id_type drone_id = 0; drone_id < g_uav_total; drone_id++)
        {
          if (!ros::service::call("/uav" + std::to_string(drone_id) + "/remove_drone_goal", remove_drone_goal_srv))
          {
            ros_error("p" + std::to_string(req.executor_id) + ": Cannot call service "
                      + ros::names::resolve("/uav" + std::to_string(drone_id) + "/remove_drone_goal"));
          }
        }

        done = true;

        ros_warn("p" + std::to_string(req.executor_id) + ": ABORTED");
        std::lock_guard<std::mutex> lock(g_feedback_pub_mutex_);
        g_feedback_pub.publish(feedback);
      });
      break;

    case urs_wearable::Feedback::STATUS_SUCCEEDED:
      g_kb.executor_map_.update_fn(req.executor_id, [&feedback, &req, &done](struct Executor& executor)
      {
        executor.executed[req.action_index] = true;
        executor.executed_total++;

        // Update KB
        const urs_wearable::Action& action = executor.actions[req.action_index];
        urs_wearable::Predicate effect;
        std::vector<urs_wearable::Predicate> effects;

        switch (action.type)
        {
          case urs_wearable::Action::TYPE_ASCEND:
            effect.type = urs_wearable::Predicate::TYPE_AT;
            effect.at.d.value = action.ascend.d.value;
            effect.at.l.value = action.ascend.l0.value;
            effect.truth_value = false;
            effects.push_back(effect);

            effect.at.d.value = action.ascend.d.value;
            effect.at.l.value = action.ascend.l1.value;
            effect.truth_value = true;
            effects.push_back(effect);

            ros_warn("p" + std::to_string(req.executor_id) + ": SUCCEEDED "
                     + action.ascend.NAME + "("
                     + std::to_string(action.ascend.d.value) + ","
                     + std::to_string(action.ascend.l0.value) + ","
                     + std::to_string(action.ascend.l1.value) + ")");
            break;

          case urs_wearable::Action::TYPE_DESCEND:
            effect.type = urs_wearable::Predicate::TYPE_AT;
            effect.at.d.value = action.descend.d.value;
            effect.at.l.value = action.descend.l0.value;
            effect.truth_value = false;
            effects.push_back(effect);

            effect.at.d.value = action.descend.d.value;
            effect.at.l.value = action.descend.l1.value;
            effect.truth_value = true;
            effects.push_back(effect);

            ros_warn("p" + std::to_string(req.executor_id) + ": SUCCEEDED "
                     + action.descend.NAME + "("
                     + std::to_string(action.descend.d.value) + ","
                     + std::to_string(action.descend.l0.value) + ","
                     + std::to_string(action.descend.l1.value) + ")");
            break;

          case urs_wearable::Action::TYPE_GATHER:
            effect.type = urs_wearable::Predicate::TYPE_AT;
            effect.at.d.value = action.gather.d.value;
            effect.at.l.value = action.gather.l0.value;
            effect.truth_value = false;
            effects.push_back(effect);

            effect.at.d.value = action.gather.d.value;
            effect.at.l.value = action.gather.l1.value;
            effect.truth_value = true;
            effects.push_back(effect);

            ros_warn("p" + std::to_string(req.executor_id) + ": SUCCEEDED "
                     + action.gather.NAME + "("
                     + std::to_string(action.gather.d.value) + ","
                     + std::to_string(action.gather.l0.value) + ","
                     + std::to_string(action.gather.l1.value) + ")");
            break;

          case urs_wearable::Action::TYPE_LAND:
            effect.type = urs_wearable::Predicate::TYPE_HOVERED;
            effect.hovered.d.value = action.land.d.value;
            effect.truth_value = false;
            effects.push_back(effect);

            ros_warn("p" + std::to_string(req.executor_id) + ": SUCCEEDED "
                     + action.land.NAME + "("
                     + std::to_string(action.land.d.value) + ")");
            break;

          case urs_wearable::Action::TYPE_MOVE:
            effect.type = urs_wearable::Predicate::TYPE_AT;
            effect.at.d.value = action.move.d.value;
            effect.at.l.value = action.move.l0.value;
            effect.truth_value = false;
            effects.push_back(effect);

            effect.at.d.value = action.move.d.value;
            effect.at.l.value = action.move.l1.value;
            effect.truth_value = true;
            effects.push_back(effect);

            ros_warn("p" + std::to_string(req.executor_id) + ": SUCCEEDED "
                     + action.move.NAME + "("
                     + std::to_string(action.move.d.value) + ","
                     + std::to_string(action.move.l0.value) + ","
                     + std::to_string(action.move.l1.value) + ")");
            break;

          case urs_wearable::Action::TYPE_SCAN:
            effect.type = urs_wearable::Predicate::TYPE_AT;
            effect.at.d.value = action.scan.d.value;
            effect.at.l.value = action.scan.l.value;
            effect.truth_value = false;
            effects.push_back(effect);

            effect.at.d.value = action.scan.d.value;
            effect.at.l.value = addLocation(ros::topic::waitForMessage<geometry_msgs::PoseStamped>
              (g_uav_ns + std::to_string(action.scan.d.value) + "/ground_truth_to_tf/pose")->pose);
            effect.truth_value = true;
            effects.push_back(effect);

            effect.type = urs_wearable::Predicate::TYPE_SCANNED;
            effect.scanned.d.value = action.scan.d.value;
            effect.scanned.a.value = action.scan.a.value;
            effect.truth_value = true;
            effects.push_back(effect);

            ros_warn("p" + std::to_string(req.executor_id) + ": SUCCEEDED "
                     + action.scan.NAME + "("
                     + std::to_string(action.scan.d.value) + ","
                     + std::to_string(action.scan.l.value) + ","
                     + std::to_string(action.scan.a.value) + ")");
          break;

          case urs_wearable::Action::TYPE_TAKEOFF:
            effect.type = urs_wearable::Predicate::TYPE_HOVERED;
            effect.hovered.d.value = action.takeoff.d.value;
            effect.truth_value = true;
            effects.push_back(effect);

            ros_warn("p" + std::to_string(req.executor_id) + ": SUCCEEDED "
                     + action.takeoff.NAME + "("
                     + std::to_string(action.takeoff.d.value) + ")");
            break;

          default:
            ros_error("p" + std::to_string(req.executor_id) + ": Unrecognized action type");
        }

        g_kb.upsertPredicates(req.executor_id, effects);

        if (executor.executed_total == executor.actions.size())
        {
          done = true;

          ros_warn("p" + std::to_string(req.executor_id) + ": SUCCEEDED");
          std::lock_guard<std::mutex> lock(g_feedback_pub_mutex_);
          g_feedback_pub.publish(feedback);
        }
      });
      break;

    default:
      ros_error("setDroneActionStatusService() received unexpected status");
  }

  if (done)
  {
    g_kb.unregisterExecutor(req.executor_id);
  }

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
    if (loc.second.position.x >= pose_sw.position.x
        && loc.second.position.x <= pose_ne.position.x
        && loc.second.position.y >= pose_sw.position.y
        && loc.second.position.y <= pose_ne.position.y
        && loc.second.position.z >= pose_sw.position.z - 0.5
        && loc.second.position.z <= pose_sw.position.z + 0.5)
    {
      pred.in.l.value = loc.first;
      pred.in.a.value = area_id;

      aux_preds.push_back(pred);
    }
  }
  loc_map_lt.unlock();
  g_kb.upsertPredicates(0, aux_preds);

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

bool getStateService(urs_wearable::GetState::Request& req, urs_wearable::GetState::Response& res)
{
  g_kb.publish();
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
  // Log the request
  ROS_INFO_STREAM("Received a set_goal request" << std::endl
                  << "player_id: " << std::to_string(req.player_id) << std::endl
                  << "goal: " << KnowledgeBase::getPredicateString(req.goal) << std::endl
                  << "(returned) executor_id: " << std::to_string(res.executor_id));

  res.executor_id = g_kb.registerExecutor();
  std::thread plan_and_execute_thread(planAndExecute, res.executor_id, std::move(req.goal));
  plan_and_execute_thread.detach();

  return true;
}

void land(drone_id_type drone_id)
{
  urs_wearable::Predicate pred;
  pred.type = urs_wearable::Predicate::TYPE_LOW_BATTERY;
  pred.truth_value = true;
  pred.low_battery.d.value = drone_id;
  g_kb.upsertPredicates(0, std::vector<urs_wearable::Predicate>{pred});

  actionlib::SimpleActionClient<urs_wearable::DroneAction> ac(g_uav_ns + std::to_string(drone_id) + "/action/drone", true);
  ac.waitForServer();

  std_srvs::Empty empty_srv;
  ros::service::call(g_uav_ns + std::to_string(drone_id) + "/clear_drone_goal", empty_srv);

  urs_wearable::DroneGoal goal;
  goal.action_type = urs_wearable::DroneGoal::TYPE_LAND;
  ac.sendGoal(goal);

  ac.waitForResult();
  if (ac.getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    pred.type = urs_wearable::Predicate::TYPE_HOVERED;
    pred.truth_value = false;
    pred.hovered.d.value = drone_id;
    g_kb.upsertPredicates(0, std::vector<urs_wearable::Predicate>{pred});
  }
}

void batteryCB(const std_msgs::StringConstPtr& s)
{
  std::vector<std::string> tokens = tokenizeString(s->data, " {':,}");
  try
  {
    if (g_unity_drone_id_index == -1 || g_unity_battery_value_index == -1)
    {
      for (size_t i = 0; i < tokens.size(); i++)
      {
        if (tokens[i] == "drone_id")
        {
          g_unity_drone_id_index = i + 1;
        }
        else if (tokens[i] == "battery_value")
        {
          g_unity_battery_value_index = i + 1;
        }
      }
    }

    drone_id_type drone_id = std::stoi(tokens[g_unity_drone_id_index]);
    int battery_value = std::stoi(tokens[g_unity_battery_value_index]);

    // Update the KB and land the drone if its battery value is less than a specified value
    if (battery_value < 10 && !g_drone_battery[drone_id].second)
    {
      g_drone_battery[drone_id].second = true;

      std::thread land_thread(land, drone_id);
      land_thread.detach();
    }
    else if (battery_value >= 10 && g_drone_battery[drone_id].second)
    {
      g_drone_battery[drone_id].second = false;

      urs_wearable::Predicate pred;
      pred.type = urs_wearable::Predicate::TYPE_LOW_BATTERY;
      pred.truth_value = false;
      pred.low_battery.d.value = drone_id;
      g_kb.upsertPredicates(0, std::vector<urs_wearable::Predicate>{pred});
    }

    if (battery_value != g_drone_battery[drone_id].first)
    {
      g_drone_battery[drone_id].first = battery_value;

      // Log the change of battery level
      ROS_INFO_STREAM("Battery" << std::endl
                      << "drone_id: " << std::to_string(drone_id) << std::endl
                      << "battery: " << std::to_string(battery_value));
    }
  }
  catch (std::exception& e)
  {
    ros_error("Cannot extract battery level message: " + s->data);
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "exec_monitor");
  ros::NodeHandle nh;

  retrieve("~uav_total", g_uav_total);
  retrieve("~uav_ns", g_uav_ns);

  // Set KB connection with a planner
  retrieve("~domain_file", g_kb.domain_file);
  retrieve("~planner_command", g_kb.planner_command);
  retrieve("~problem_path", g_kb.problem_path);
  retrieve("~tmp_path", g_kb.tmp_path);

  ros::Publisher state_pub = nh.advertise<urs_wearable::State>("urs_wearable/state", 10, true);
  g_kb.setStatePub(&state_pub);

  ros::ServiceClient set_drone_action_status_client = nh.serviceClient<urs_wearable::SetDroneActionStatus>("urs_wearable/set_drone_action_status");
  g_kb.setDroneActionStatusClient(&set_drone_action_status_client);

  // Set feedback publisher
  g_feedback_pub = nh.advertise<urs_wearable::Feedback>("urs_wearable/feedback", 10, true);

  // Set initial state
  std::vector<urs_wearable::Predicate> initial_state;

  urs_wearable::Predicate pred_at;
  pred_at.type = urs_wearable::Predicate::TYPE_AT;
  pred_at.truth_value = true;

  urs_wearable::Predicate pred_hovered;
  pred_hovered.type = urs_wearable::Predicate::TYPE_HOVERED;
  pred_hovered.truth_value = false;

  urs_wearable::Predicate pred_low_battery;
  pred_low_battery.type = urs_wearable::Predicate::TYPE_LOW_BATTERY;
  pred_low_battery.truth_value = false;

  g_drone_battery.reserve(g_uav_total);

  for (int i = 0; i < g_uav_total; i++)
  {
    geometry_msgs::PoseStamped::ConstPtr pose_stamped =
        ros::topic::waitForMessage<geometry_msgs::PoseStamped>(g_uav_ns + std::to_string(i) + "/ground_truth_to_tf/pose");

    pred_at.at.d.value = i;
    pred_at.at.l.value = addLocation(pose_stamped->pose);
    initial_state.push_back(pred_at);

    pred_hovered.hovered.d.value = i;
    initial_state.push_back(pred_hovered);

    pred_low_battery.low_battery.d.value = i;
    initial_state.push_back(pred_low_battery);

    // Set battery level vector to be used for logging purpose
    g_drone_battery.push_back(std::make_pair(100, false));
  }
  g_kb.upsertPredicates(0, initial_state);

  // Subscribe to topics
  ros::Subscriber battery_subscribe = nh.subscribe("/w_battery_value", 1, batteryCB);

  // Advertise services
  ros::ServiceServer add_area_service = nh.advertiseService("urs_wearable/add_area", addAreaService);
  ros::ServiceServer add_location_service = nh.advertiseService("urs_wearable/add_location", addLocationService);
  ros::ServiceServer get_state_service = nh.advertiseService("urs_wearable/get_state", getStateService);
  ros::ServiceServer remove_area_service = nh.advertiseService("urs_wearable/remove_area",removeAreaService);
  ros::ServiceServer remove_location_service = nh.advertiseService("urs_wearable/remove_location", removeLocationService);
  ros::ServiceServer set_drone_action_result_service = nh.advertiseService("urs_wearable/set_drone_action_status", setDroneActionStatusService);
  ros::ServiceServer set_goal_service = nh.advertiseService("urs_wearable/set_goal", setGoalService);

  ros_info("Waiting for connections from wearable devices...");

  ros::spin();

  return EXIT_SUCCESS;
}
