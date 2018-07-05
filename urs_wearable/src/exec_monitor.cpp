#include <chrono>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include "libcuckoo/cuckoohash_map.hh"
#include <ros/ros.h>

#include "urs_wearable/controller.h"
#include "urs_wearable/knowledge_base.h"
#include "urs_wearable/navigator.h"
#include "urs_wearable/Action.h"
#include "urs_wearable/DroneAction.h"
#include "urs_wearable/Feedback.h"
#include "urs_wearable/GetState.h"
#include "urs_wearable/LocationAdd.h"
#include "urs_wearable/LocationRemove.h"
#include "urs_wearable/PoseEuler.h"
#include "urs_wearable/SetGoal.h"

const unsigned int N_UAV = 4;
const std::string PLANNER_SERVICE_NAME = "/cpa/get_plan";

Controller g_controller[N_UAV];
//Navigator navigator[N_UAV];

KnowledgeBase g_kb("urs_problem", "urs", PLANNER_SERVICE_NAME);

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

  feedback.executor_id = executor_id;
  feedback.status = urs_wearable::Feedback::STATUS_PENDING;
  feedback_pub.publish(feedback);

  std::vector<std::string> plan;
  g_kb.getPlan(executor_id, req.goal, plan);

  if (!plan.empty())
  {
    std::vector<urs_wearable::Action> actions = g_kb.parsePlan(plan);
    feedback.status = urs_wearable::Feedback::STATUS_ACTIVE;

    for (const auto& action : actions)
    {
      std::vector<urs_wearable::Predicate> effects;
      feedback.current_action = action;

      switch (action.type)
      {
        case urs_wearable::Action::TYPE_ACTIVE_REGION_INSERT:
        {
          feedback_pub.publish(feedback);

          // Add the effects of the action to the list
          urs_wearable::Predicate pred;
          pred.type = urs_wearable::Predicate::TYPE_ACTIVE_REGION;
          pred.predicate_active_region.location_id_sw.value = action.action_active_region_insert.location_id_sw.value;
          pred.predicate_active_region.location_id_ne.value = action.action_active_region_insert.location_id_ne.value;
          pred.predicate_active_region.truth_value = true;
          effects.push_back(pred);
        }
          break;

        case urs_wearable::Action::TYPE_ACTIVE_REGION_UPDATE:
        {
          feedback_pub.publish(feedback);

          // Add the effects of the action to the list
          urs_wearable::Predicate pred;
          pred.type = urs_wearable::Predicate::TYPE_ACTIVE_REGION;
          pred.predicate_active_region.location_id_sw.value = action.action_active_region_update.location_id_sw_old.value;
          pred.predicate_active_region.location_id_ne.value = action.action_active_region_update.location_id_ne_old.value;
          pred.predicate_active_region.truth_value = false;
          effects.push_back(pred);

          pred.predicate_active_region.location_id_sw.value = action.action_active_region_update.location_id_sw_new.value;
          pred.predicate_active_region.location_id_ne.value = action.action_active_region_update.location_id_ne_new.value;
          pred.predicate_active_region.truth_value = true;
          effects.push_back(pred);
        }
          break;

        case urs_wearable::Action::TYPE_FLY_ABOVE:
          break;

        case urs_wearable::Action::TYPE_FLY_TO:
          break;

        case urs_wearable::Action::TYPE_KEY_ADD:
        {
          feedback_pub.publish(feedback);

          // Add the effects of the action to the list
          urs_wearable::Predicate pred;
          pred.type = urs_wearable::Predicate::TYPE_KEY_AT;
          pred.predicate_key_at.key_id.value = action.action_key_add.key_id.value;
          pred.predicate_key_at.location_id.value = action.action_key_add.location_id.value;
          pred.predicate_key_at.truth_value = true;
          effects.push_back(pred);
        }
          break;

        case urs_wearable::Action::TYPE_KEY_PICK:
          break;

        case urs_wearable::Action::TYPE_TAKE_OFF:
        {
          urs_wearable::DroneGoal goal;
          goal.action_type = urs_wearable::DroneGoal::TYPE_GOTO;
          goal.pose = g_controller[action.action_take_off.drone_id.value].getPose();
          goal.pose.position.z += 2.0;

          actionlib::SimpleActionClient<urs_wearable::DroneAction> ac(
              "/uav" + std::to_string(action.action_take_off.drone_id.value) + "/action_server", true);
          ac.waitForServer();
          ac.sendGoal(goal);

          feedback_pub.publish(feedback);

          if (ac.waitForResult(ros::Duration(30.0)))
          {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
          }
          else
          {
            ROS_INFO("Action did not finish before the time out");
          }

          // Add the effects of the action to the list
          urs_wearable::Predicate pred;
          pred.type = urs_wearable::Predicate::TYPE_TOOK_OFF;
          pred.predicate_took_off.drone_id.value = action.action_take_off.drone_id.value;
          pred.predicate_took_off.truth_value = true;
          effects.push_back(pred);
        }
          break;
      }

      // Upsert the effects of the action
      g_kb.upsertPredicates(effects);
    }
  }
  else
  {
    feedback.status = urs_wearable::Feedback::STATUS_REJECTED;
    feedback_pub.publish(feedback);
  }

  g_kb.unregisterExecutor(executor_id);
  feedback_pub.shutdown();
}

bool getState(urs_wearable::GetState::Request& req, urs_wearable::GetState::Response& res)
{
  g_kb.publish();
  return true;
}

bool locationAdd(urs_wearable::LocationAdd::Request& req, urs_wearable::LocationAdd::Response& res)
{
  res.location_id = g_kb.location_table_.insert(req.pose);
  return true;
}

bool locationRemove(urs_wearable::LocationRemove::Request& req, urs_wearable::LocationRemove::Response& res)
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

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "exec_monitor");
  ros::NodeHandle nh;

  if (N_UAV == 0)
  {
    ROS_ERROR("No UAV to control");
    exit(EXIT_SUCCESS);
  }

  // Initialize controllers, navigators, and action clients
  for (unsigned int i = 0; i < N_UAV; i++)
  {
    if (!g_controller[i].setNamespace("/uav" + std::to_string(i)))
    {
      ROS_ERROR("Error in setting up controller %u", i);
      exit(EXIT_FAILURE);
    }

//    navigator[i].init();
//    navigator[i].setNamespace("/uav" + std::to_string(i));
  }

  // Set KB state publisher
  ros::Publisher state_pub = nh.advertise<urs_wearable::State>("/urs_wearable/state", 100);
  g_kb.setStatePub(&state_pub);

  // Set initial state
  std::vector<urs_wearable::Predicate> initial_state;
  urs_wearable::Predicate pred_took_off;
  pred_took_off.type = urs_wearable::Predicate::TYPE_TOOK_OFF;
  pred_took_off.predicate_took_off.truth_value = false;

  urs_wearable::Predicate pred_drone_at;
  pred_drone_at.type = urs_wearable::Predicate::TYPE_DRONE_AT;
  pred_drone_at.predicate_drone_at.truth_value = true;

  for (unsigned int i = 0; i < N_UAV; i++)
  {
    pred_took_off.predicate_took_off.drone_id.value = i;
    initial_state.push_back(pred_took_off);

    pred_drone_at.predicate_drone_at.drone_id.value = i;
    pred_drone_at.predicate_drone_at.location_id.value = g_kb.location_table_.insert(g_controller[i].getPose());
    initial_state.push_back(pred_drone_at);
  }
  g_kb.upsertPredicates(initial_state);

  // Advertise services
  ros::ServiceServer get_state_service = nh.advertiseService("/urs_wearable/get_statel", getState);
  ros::ServiceServer location_add_service = nh.advertiseService("/urs_wearable/location_add", locationAdd);
  ros::ServiceServer location_remove_service = nh.advertiseService("/urs_wearable/location_remove", locationRemove);
  ros::ServiceServer set_goal_service = nh.advertiseService("/urs_wearable/set_goal", setGoal);

  ROS_INFO("Waiting for connections from wearable devices...");

  ros::spin();

  // Clean up
  get_state_service.shutdown();
  location_add_service.shutdown();
  location_remove_service.shutdown();
  set_goal_service.shutdown();
}
