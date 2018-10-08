#include <chrono>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <libcuckoo/cuckoohash_map.hh>
#include <ros/ros.h>
#include <urs_wearable/Action.h>
#include <urs_wearable/DroneAction.h>
#include <urs_wearable/Feedback.h>
#include <urs_wearable/GetPose.h>
#include <urs_wearable/GetState.h>
#include <urs_wearable/LocationAdd.h>
#include <urs_wearable/LocationRemove.h>
#include <urs_wearable/PoseEuler.h>
#include <urs_wearable/SetGoal.h>

#include <urs_wearable/SetDest.h>
#include <urs_wearable/Gather.h>
#include <urs_wearable/Scan.h>

#include "urs_wearable/common.h"
#include "urs_wearable/knowledge_base.h"

// We use uint8_t here to match with the type of 'value' in ObjectDroneID.msg
typedef std::uint8_t drone_id_type;

const std::string PLANNER_SERVICE_NAME = "/cpa/get_plan";

KnowledgeBase g_kb("urs_problem", "urs", PLANNER_SERVICE_NAME);

bool isWithinActiveRegion(urs_wearable::PoseEuler& pose, const LocationTable::location_id_type location_id, std::string& feedback_message)
{
  std::vector<urs_wearable::Predicate> active_region_preds = g_kb.getPredicateList(urs_wearable::Predicate::TYPE_ACTIVE_REGION);

  // Check the validity of predicate active_region
  if (active_region_preds.size() == 0)
  {
    feedback_message = "Predicate active_region is not in the knowledge base";
    return false;
  }
  if (active_region_preds.size() > 1)
  {
    feedback_message = "There cannot be more than one active_region predicate";
    return false;
  }

  // Get poses of action region
  urs_wearable::Predicate pred_active_region = active_region_preds.back();
  urs_wearable::PoseEuler pose_active_region_sw;
  if (!g_kb.location_table_.map_.find(pred_active_region.predicate_active_region.location_id_sw.value, pose_active_region_sw))
  {
    feedback_message = "Cannot find location id " + std::to_string(pred_active_region.predicate_active_region.location_id_sw.value) + " in the location table";
    return false;
  }

  urs_wearable::PoseEuler pose_active_region_ne;
  if (!g_kb.location_table_.map_.find(pred_active_region.predicate_active_region.location_id_ne.value, pose_active_region_ne))
  {
    feedback_message = "Cannot find location id " + std::to_string(pred_active_region.predicate_active_region.location_id_ne.value) + " in the location table";
    return false;
  }

  // Get the pose to fly to
  if (!g_kb.location_table_.map_.find(location_id, pose))
  {
    feedback_message = "Cannot find location id " + std::to_string(location_id) + " in the location table";
    return false;
  }

  // Check that the pose to fly to is within active region
  if (pose.position.x < pose_active_region_sw.position.x || pose.position.x > pose_active_region_ne.position.x
      || pose.position.y < pose_active_region_sw.position.y || pose.position.y > pose_active_region_ne.position.y
      || pose.position.z < pose_active_region_sw.position.z || pose.position.z > pose_active_region_ne.position.z)
  {
    feedback_message = "The pose to fly above is outside of active region";
    return false;
  }

  return true;
}

void executor(urs_wearable::SetGoal::Request req)
{
  if (req.feedback_topic_name.empty())
  {
    req.feedback_topic_name = "/urs_wearable/feedback_dump";
  }

  KnowledgeBase::executor_id_type executor_id = g_kb.registerExecutor();

  ros::NodeHandle nh;
  ros::Publisher feedback_pub = nh.advertise<urs_wearable::Feedback>(req.feedback_topic_name, 10);
  urs_wearable::Feedback feedback;

  ROS_INFO("Executor %u: PENDING", executor_id);
  feedback.executor_id = executor_id;
  feedback.status = urs_wearable::Feedback::STATUS_PENDING;
  feedback_pub.publish(feedback);
  ros::spinOnce();

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
        case urs_wearable::Action::TYPE_ACTIVE_REGION_UPDATE:
        {
          ROS_INFO("Executor %u: ACTIVE - action %s", executor_id, urs_wearable::ActionActiveRegionUpdate::NAME.c_str());
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          // Check the validity of location_id_sw_new and location_id_ne_new
          urs_wearable::PoseEuler pose_sw_new;
          urs_wearable::PoseEuler pose_ne_new;
          LocationTable::location_id_type location_id_sw_new = actions_it->action_active_region_update.location_id_sw_new.value;
          LocationTable::location_id_type location_id_ne_new = actions_it->action_active_region_update.location_id_ne_new.value;

          if (!g_kb.location_table_.map_.find(location_id_sw_new, pose_sw_new))
          {
            ROS_WARN("Executor %u: ABORTED", executor_id);
            feedback.status = urs_wearable::Feedback::STATUS_ABORTED;
            feedback.message = "Cannot find location id " + std::to_string(location_id_sw_new) + " in the location table";
            feedback_pub.publish(feedback);
            ros::spinOnce();

            g_kb.unregisterExecutor(executor_id);
            feedback_pub.shutdown();
            return;
          }

          if (!g_kb.location_table_.map_.find(location_id_ne_new, pose_ne_new))
          {
            ROS_WARN("Executor %u: ABORTED", executor_id);
            feedback.status = urs_wearable::Feedback::STATUS_ABORTED;
            feedback.message = "Cannot find location id " + std::to_string(location_id_ne_new) + " in the location table";
            feedback_pub.publish(feedback);
            ros::spinOnce();

            g_kb.unregisterExecutor(executor_id);
            feedback_pub.shutdown();
            return;
          }

          if (pose_sw_new.position.x > pose_ne_new.position.x
              || pose_sw_new.position.y > pose_ne_new.position.y
              || pose_sw_new.position.z > pose_ne_new.position.z)
          {
            ROS_WARN("Executor %u: ABORTED", executor_id);
            feedback.status = urs_wearable::Feedback::STATUS_ABORTED;
            feedback.message = "The new position of location_id_sw is not in the south-west of the new position of location_id_ne";
            feedback_pub.publish(feedback);
            ros::spinOnce();

            g_kb.unregisterExecutor(executor_id);
            feedback_pub.shutdown();
            return;
          }

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_ACTIVE_REGION;
          effect.predicate_active_region.location_id_sw.value = actions_it->action_active_region_update.location_id_sw_old.value;
          effect.predicate_active_region.location_id_ne.value = actions_it->action_active_region_update.location_id_ne_old.value;
          effect.predicate_active_region.truth_value = false;
          effects.push_back(effect);

          effect.predicate_active_region.location_id_sw.value = location_id_sw_new;
          effect.predicate_active_region.location_id_ne.value = location_id_ne_new;
          effect.predicate_active_region.truth_value = true;
          effects.push_back(effect);
        }
        break;

//        case urs_wearable::Action::TYPE_ADD_LOCATION:
//        {
//          ROS_INFO("Executor %u: ACTIVE - action %s", executor_id, urs_wearable::ActionAddLocation::NAME.c_str());
//          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
//          feedback.current_action = *actions_it;
//          feedback_pub.publish(feedback);
//          ros::spinOnce();
//
//          // Add the effects of the action to the list
//          urs_wearable::Predicate effect;
//          effect.type = urs_wearable::Predicate::TYPE_IS_LOCATION;
//          effect.predicate_is_location.location_id.value = actions_it->action_add_location.location_id.value;
//          effect.predicate_is_location.truth_value = true;
//          effects.push_back(effect);
//
//          effect.type = urs_wearable::Predicate::TYPE_IS_OCCUPIED;
//          effect.predicate_is_occupied.location_id.value = actions_it->action_add_location.location_id.value;
//          effect.predicate_is_occupied.truth_value = false;
//          effects.push_back(effect);
//        }
//        break;

        case urs_wearable::Action::TYPE_FLY_ABOVE:
        {
          ROS_INFO("Executor %u: ACTIVE - action %s", executor_id, urs_wearable::ActionFlyAbove::NAME.c_str());
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          // Get the pose to fly to
          urs_wearable::PoseEuler pose_to;
          LocationTable::location_id_type location_id_to = actions_it->action_fly_above.location_id_to.value;

          if (!isWithinActiveRegion(pose_to, location_id_to, feedback.message))
          {
            ROS_WARN("Executor %u: ABORTED", executor_id);
            feedback.status = urs_wearable::Feedback::STATUS_ABORTED;
            feedback_pub.publish(feedback);
            ros::spinOnce();

            g_kb.unregisterExecutor(executor_id);
            feedback_pub.shutdown();
            return;
          }

          // Add the height to fly above
          double fly_above_height = 1.0;
          pose_to.position.z += fly_above_height;

          require_drone_action = true;
          drone_id = actions_it->action_fly_above.drone_id.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_POSE;
          goal.pose = pose_to;
          goal.set_orientation = false;

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_DRONE_ABOVE;
          effect.predicate_drone_above.drone_id.value = drone_id;
          effect.predicate_drone_above.location_id.value = actions_it->action_fly_above.location_id_from.value;
          effect.predicate_drone_above.truth_value = false;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_DRONE_AT;
          effect.predicate_drone_at.drone_id.value = drone_id;
          effect.predicate_drone_at.location_id.value = actions_it->action_fly_above.location_id_from.value;
          effect.predicate_drone_at.truth_value = false;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_DRONE_ABOVE;
          effect.predicate_drone_above.drone_id.value = drone_id;
          effect.predicate_drone_above.location_id.value = location_id_to;
          effect.predicate_drone_above.truth_value = true;
          effects.push_back(effect);

//          effect.type = urs_wearable::Predicate::TYPE_IS_OCCUPIED;
//          effect.predicate_is_occupied.location_id.value = actions_it->action_fly_above.location_id_from.value;
//          effect.predicate_is_occupied.truth_value = false;
//          effects.push_back(effect);
//
//          effect.type = urs_wearable::Predicate::TYPE_IS_OCCUPIED;
//          effect.predicate_is_occupied.location_id.value = actions_it->action_fly_above.location_id_to.value;
//          effect.predicate_is_occupied.truth_value = true;
//          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_FLY_TO:
        {
          ROS_INFO("Executor %u: ACTIVE - action %s", executor_id, urs_wearable::ActionFlyTo::NAME.c_str());
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          // Get the pose to fly to
          urs_wearable::PoseEuler pose_to;
          LocationTable::location_id_type location_id_to = actions_it->action_fly_to.location_id_to.value;

          if (!isWithinActiveRegion(pose_to, location_id_to, feedback.message))
          {
            ROS_WARN("Executor %u: ABORTED", executor_id);
            feedback.status = urs_wearable::Feedback::STATUS_ABORTED;
            feedback_pub.publish(feedback);
            ros::spinOnce();

            g_kb.unregisterExecutor(executor_id);
            feedback_pub.shutdown();
            return;
          }

          require_drone_action = true;
          drone_id = actions_it->action_fly_to.drone_id.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_POSE;
          goal.pose = pose_to;
          goal.set_orientation = false;

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_DRONE_ABOVE;
          effect.predicate_drone_above.drone_id.value = drone_id;
          effect.predicate_drone_above.location_id.value = actions_it->action_fly_to.location_id_from.value;
          effect.predicate_drone_above.truth_value = false;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_DRONE_AT;
          effect.predicate_drone_at.drone_id.value = drone_id;
          effect.predicate_drone_at.location_id.value = actions_it->action_fly_to.location_id_from.value;
          effect.predicate_drone_at.truth_value = false;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_DRONE_AT;
          effect.predicate_drone_at.drone_id.value = drone_id;
          effect.predicate_drone_at.location_id.value = location_id_to;
          effect.predicate_drone_at.truth_value = true;
          effects.push_back(effect);

//          effect.type = urs_wearable::Predicate::TYPE_IS_OCCUPIED;
//          effect.predicate_is_occupied.location_id.value = actions_it->action_fly_to.location_id_from.value;
//          effect.predicate_is_occupied.truth_value = false;
//          effects.push_back(effect);
//
//          effect.type = urs_wearable::Predicate::TYPE_IS_OCCUPIED;
//          effect.predicate_is_occupied.location_id.value = actions_it->action_fly_to.location_id_to.value;
//          effect.predicate_is_occupied.truth_value = true;
//          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_KEY_ADD:
        {
          ROS_INFO("Executor %u: ACTIVE - action %s", executor_id, urs_wearable::ActionKeyAdd::NAME.c_str());
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
//          effect.type = urs_wearable::Predicate::TYPE_KEY_AT;
//          effect.predicate_key_at.key_id.value = actions_it->action_key_add.key_id.value;
//          effect.predicate_key_at.location_id.value = actions_it->action_key_add.location_id.value;
//          effect.predicate_key_at.truth_value = true;
//          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_KEY_PICKED;
          effect.predicate_key_picked.key_id.value = actions_it->action_key_add.key_id.value;
          effect.predicate_key_picked.truth_value = false;
          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_KEY_PICK:
        {
          ROS_INFO("Executor %u: ACTIVE - action %s", executor_id, urs_wearable::ActionKeyPick::NAME.c_str());
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          // Get the pose to fly to
          urs_wearable::PoseEuler pose_to;
          LocationTable::location_id_type key_location_id = actions_it->action_key_pick.key_location_id.value;

          if (!isWithinActiveRegion(pose_to, key_location_id, feedback.message))
          {
            ROS_WARN("Executor %u: ABORTED", executor_id);
            feedback.status = urs_wearable::Feedback::STATUS_ABORTED;
            feedback_pub.publish(feedback);
            ros::spinOnce();

            g_kb.unregisterExecutor(executor_id);
            feedback_pub.shutdown();
            return;
          }

          // Add the height to fly above
          double fly_above_height = 1.0;
          pose_to.position.z += fly_above_height;

          require_drone_action = true;
          drone_id = actions_it->action_key_pick.drone_id.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_POSE;
          goal.pose = pose_to;
          goal.set_orientation = false;

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_DRONE_ABOVE;
          effect.predicate_drone_above.drone_id.value = drone_id;
          effect.predicate_drone_above.location_id.value = actions_it->action_key_pick.drone_location_id.value;
          effect.predicate_drone_above.truth_value = false;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_DRONE_AT;
          effect.predicate_drone_at.drone_id.value = drone_id;
          effect.predicate_drone_at.location_id.value = actions_it->action_key_pick.drone_location_id.value;
          effect.predicate_drone_at.truth_value = false;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_DRONE_ABOVE;
          effect.predicate_drone_above.drone_id.value = drone_id;
          effect.predicate_drone_above.location_id.value = key_location_id;
          effect.predicate_drone_above.truth_value = true;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_KEY_PICKED;
          effect.predicate_key_picked.key_id.value = actions_it->action_key_pick.key_id.value;
          effect.predicate_key_picked.truth_value = true;
          effects.push_back(effect);

          effect.type = urs_wearable::Predicate::TYPE_KEY_WITH;
          effect.predicate_key_with.key_id.value = actions_it->action_key_pick.key_id.value;
          effect.predicate_key_with.drone_id.value = drone_id;
          effect.predicate_key_with.truth_value = true;
          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_LAND:
        {
          ROS_INFO("Executor %u: ACTIVE - action %s", executor_id, urs_wearable::ActionLand::NAME.c_str());
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          require_drone_action = true;
          drone_id = actions_it->action_land.drone_id.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_LANDING;

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_TOOK_OFF;
          effect.predicate_took_off.drone_id.value = actions_it->action_land.drone_id.value;
          effect.predicate_took_off.truth_value = false;
          effects.push_back(effect);
        }
        break;

        case urs_wearable::Action::TYPE_TAKE_OFF:
        {
          ROS_INFO("Executor %u: ACTIVE - action %s", executor_id, urs_wearable::ActionTakeOff::NAME.c_str());
          feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;
          feedback.current_action = *actions_it;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          require_drone_action = true;
          drone_id = actions_it->action_take_off.drone_id.value;
          goal.action_type = urs_wearable::DroneGoal::TYPE_TAKEOFF;

          // Add the effects of the action to the list
          urs_wearable::Predicate effect;
          effect.type = urs_wearable::Predicate::TYPE_TOOK_OFF;
          effect.predicate_took_off.drone_id.value = actions_it->action_take_off.drone_id.value;
          effect.predicate_took_off.truth_value = true;
          effects.push_back(effect);
        }
        break;

        default:
        {
          ROS_ERROR("Executor %u: Unrecognized action", executor_id);
          g_kb.unregisterExecutor(executor_id);
          feedback_pub.shutdown();
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
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
          ROS_INFO("Executor %u: REPLANNED", executor_id);
          feedback.status = urs_wearable::Feedback::STATUS_REPLANNED;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          ac.cancelGoal();
          ROS_INFO("Executor %u: Drone action finished with state: %s", executor_id, ac.getState().toString().c_str());

          if (new_plan.empty())
          {
            if (g_kb.excludeAlreadySatisfiedGoals(req.goal).empty())
            {
              ROS_WARN("Executor %u: SUCCEEDED", executor_id);
              feedback.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
              feedback_pub.publish(feedback);
              ros::spinOnce();
            }
            else
            {
              ROS_WARN("Executor %u: PREEMPTED", executor_id);
              feedback.status = urs_wearable::Feedback::STATUS_PREEMPTED;
              feedback_pub.publish(feedback);
              ros::spinOnce();
            }
            g_kb.unregisterExecutor(executor_id);
            feedback_pub.shutdown();
            return;
          }

          plan = new_plan;
          plan_it = plan.begin();
          actions = g_kb.parsePlan(new_plan);
          actions_it = actions.begin();

          continue;
        }

        ROS_INFO("Executor %u: Drone action finished with state: %s", executor_id, ac.getState().toString().c_str());
        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_WARN("Executor %u: PREEMPTED", executor_id);
          feedback.status = urs_wearable::Feedback::STATUS_PREEMPTED;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          g_kb.unregisterExecutor(executor_id);
          feedback_pub.shutdown();
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
          ROS_INFO("Executor %u: REPLANNED", executor_id);
          feedback.status = urs_wearable::Feedback::STATUS_REPLANNED;
          feedback_pub.publish(feedback);
          ros::spinOnce();

          if (new_plan.empty())
          {
            if (g_kb.excludeAlreadySatisfiedGoals(req.goal).empty())
            {
              ROS_WARN("Executor %u: SUCCEEDED", executor_id);
              feedback.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
              feedback_pub.publish(feedback);
              ros::spinOnce();
            }
            else
            {
              ROS_WARN("Executor %u: PREEMPTED", executor_id);
              feedback.status = urs_wearable::Feedback::STATUS_PREEMPTED;
              feedback_pub.publish(feedback);
              ros::spinOnce();
            }
            g_kb.unregisterExecutor(executor_id);
            feedback_pub.shutdown();
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

    ROS_WARN("Executor %u: SUCCEEDED", executor_id);
    feedback.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
    feedback_pub.publish(feedback);
    ros::spinOnce();
  }
  else if (g_kb.excludeAlreadySatisfiedGoals(req.goal).empty())
  {
    ROS_WARN("Executor %u: SUCCEEDED", executor_id);
    feedback.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
    feedback_pub.publish(feedback);
    ros::spinOnce();
  }
  else
  {
    ROS_WARN("Executor %u: REJECTED", executor_id);
    feedback.status = urs_wearable::Feedback::STATUS_REJECTED;
    feedback_pub.publish(feedback);
    ros::spinOnce();
  }

  g_kb.unregisterExecutor(executor_id);
  feedback_pub.shutdown();
}

bool getState(urs_wearable::GetState::Request& req, urs_wearable::GetState::Response& res)
{
  g_kb.publish();
  return true;
}

bool addLocation(urs_wearable::LocationAdd::Request& req, urs_wearable::LocationAdd::Response& res)
{
  res.location_id = g_kb.location_table_.insert(req.pose);
  return true;
}

bool removeLocation(urs_wearable::LocationRemove::Request& req, urs_wearable::LocationRemove::Response& res)
{
  // TODO: Remove all predicates that have the removed location id
  return true;
}

bool setGoal(urs_wearable::SetGoal::Request& req, urs_wearable::SetGoal::Response& res)
{
  std::thread thread_executor(executor, req);
  thread_executor.detach();
  return true;
}

bool scan(urs_wearable::Scan::Request& req, urs_wearable::Scan::Response& res)
{
  return true;
}

bool gather(urs_wearable::Gather::Request& req, urs_wearable::Gather::Response& res)
{
  for (int i = 0; i < req.uav_id.size(); i++)
  {
    urs_wearable::PoseEuler dest;
    dest.position = req.position;

    switch (req.uav_id[i])
    {
      case 0:
        dest.position.x += 2;
        break;
      case 1:
        dest.position.x -= 2;
        break;
      case 2:
        dest.position.y += 2;
        break;
      case 3:
        dest.position.y -= 2;
        break;
    }

    urs_wearable::SetDest set_dest_srv;
    set_dest_srv.request.dest = dest;
    ros::service::call("/uav" + std::to_string(req.uav_id[i]) + "/set_dest", set_dest_srv);
  }
  return true;
}

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "exec_monitor");
  ros::NodeHandle nh;

  // Set KB state publisher
  ros::Publisher state_pub = nh.advertise<urs_wearable::State>("/urs_wearable/state", 100);
  g_kb.setStatePub(&state_pub);

  // Set initial state
  std::vector<urs_wearable::Predicate> initial_state;
  urs_wearable::Predicate pred_active_region;
  pred_active_region.type = urs_wearable::Predicate::TYPE_ACTIVE_REGION;
  urs_wearable::PoseEuler pose_sw;
  pose_sw.position.x = -10.0;
  pose_sw.position.y = -10.0;
  pose_sw.position.z = 0.0;
  urs_wearable::PoseEuler pose_ne;
  pose_ne.position.x = 10.0;
  pose_ne.position.y = 10.0;
  pose_ne.position.z = 10.0;
  pred_active_region.predicate_active_region.location_id_sw.value = g_kb.location_table_.insert(pose_sw);
  pred_active_region.predicate_active_region.location_id_ne.value = g_kb.location_table_.insert(pose_ne);
  pred_active_region.predicate_active_region.truth_value = true;
  initial_state.push_back(pred_active_region);

//  urs_wearable::Predicate pred_is_location;
//  pred_is_location.type = urs_wearable::Predicate::TYPE_IS_LOCATION;
//  pred_is_location.predicate_is_location.truth_value = true;
//
//  urs_wearable::Predicate pred_is_occupied;
//  pred_is_occupied.type = urs_wearable::Predicate::TYPE_IS_OCCUPIED;
//  pred_is_occupied.predicate_is_occupied.truth_value = true;
//
//  for (unsigned int i = 0; i < 6; i++)
//  {
//    pred_is_location.predicate_is_location.location_id.value = i;
//    initial_state.push_back(pred_is_location);
//
//    if (i >= 2)
//    {
//      pred_is_occupied.predicate_is_occupied.location_id.value = i;
//      initial_state.push_back(pred_is_occupied);
//    }
//  }

  urs_wearable::Predicate pred_drone_at;
  pred_drone_at.type = urs_wearable::Predicate::TYPE_DRONE_AT;
  pred_drone_at.predicate_drone_at.truth_value = true;

  urs_wearable::Predicate pred_took_off;
  pred_took_off.type = urs_wearable::Predicate::TYPE_TOOK_OFF;
  pred_took_off.predicate_took_off.truth_value = false;

  int n_uav;
  retrieve("n_uav", n_uav);

  std::string uav_ns;
  retrieve("uav_ns", uav_ns);

  for (unsigned int i = 0; i < n_uav; i++)
  {
    if (!ros::service::waitForService(uav_ns + std::to_string(i) + "/get_pose", 60000))
    {
      ros_error("The controller for drone " + uav_ns + std::to_string(i) + " is not running");
      return EXIT_FAILURE;
    }

    urs_wearable::GetPose get_pose_srv;
    if (!ros::service::call(uav_ns + std::to_string(i) + "/get_pose", get_pose_srv))
    {
      ros_error("Error in calling service " + uav_ns + std::to_string(i) + "/get_pose");
      return EXIT_FAILURE;
    }

    pred_drone_at.predicate_drone_at.drone_id.value = i;
    pred_drone_at.predicate_drone_at.location_id.value = g_kb.location_table_.insert(get_pose_srv.response.pose);
    initial_state.push_back(pred_drone_at);

    pred_took_off.predicate_took_off.drone_id.value = i;
    initial_state.push_back(pred_took_off);
  }
  g_kb.upsertPredicates(initial_state);

  // Advertise services
  ros::ServiceServer add_location_service = nh.advertiseService("urs_wearable/add_location", addLocation);
  ros::ServiceServer get_state_service = nh.advertiseService("urs_wearable/get_state", getState);
  ros::ServiceServer remove_location_service = nh.advertiseService("urs_wearable/remove_location", removeLocation);
  ros::ServiceServer set_goal_service = nh.advertiseService("urs_wearable/set_goal", setGoal);

  // Advertise temporary services
  ros::ServiceServer scan_service = nh.advertiseService("urs_wearable/scan", scan);
  ros::ServiceServer gather_location_service = nh.advertiseService("urs_wearable/gather", gather);

  ros_info("Waiting for connections from wearable devices...");

  ros::spin();

  // Clean up
  add_location_service.shutdown();
  get_state_service.shutdown();
  remove_location_service.shutdown();
  set_goal_service.shutdown();

  return EXIT_SUCCESS;
}
