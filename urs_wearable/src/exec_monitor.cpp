#include <chrono>
#include <thread>

#include <actionlib/client/service_client.h>
#include "libcuckoo/cuckoohash_map.hh"
#include <ros/ros.h>

#include "urs_wearable/controller.h"
#include "urs_wearable/knowledge_base.h"
#include "urs_wearable/navigator.h"
#include "urs_wearable/Action.h"
#include "urs_wearable/ActionsAction.h"
#include "urs_wearable/Feedback.h"
#include "urs_wearable/LocationAdd.h"
#include "urs_wearable/LocationRemove.h"
#include "urs_wearable/PoseEuler.h"
#include "urs_wearable/SetGoal.h"

const unsigned int N_UAV = 4;
const std::string PLANNER_SERVICE_NAME = "/cpa/get_plan";

actionlib::SimpleActionClient<urs_wearable::ActionsAction>* g_action_client[N_UAV];
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

    for (const auto& action : actions)
    {
      switch (action.type)
      {
        case urs_wearable::Action::TYPE_ACTIVE_REGION_INSERT:
          break;

        case urs_wearable::Action::TYPE_ACTIVE_REGION_UPDATE:
          break;

        case urs_wearable::Action::TYPE_FLY_ABOVE:
          break;

        case urs_wearable::Action::TYPE_FLY_TO:
          break;

        case urs_wearable::Action::TYPE_KEY_ADD:
          break;

        case urs_wearable::Action::TYPE_KEY_PICK:
          break;

        case urs_wearable::Action::TYPE_TAKE_OFF:
          {
            urs_wearable::ActionsGoal goal;
            goal.action_type = urs_wearable::Action::TYPE_TAKE_OFF;

            auto& ac = *g_action_client[action.action_take_off.drone_id.value];
            ac.sendGoal(goal);

            bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
            if (finished_before_timeout)
            {
              actionlib::SimpleClientGoalState state = ac.getState();
              ROS_INFO("Action finished: %s",state.toString().c_str());
            }
            else
            {
              ROS_INFO("Action did not finish before the time out.");
            }

            // Upsert effects of the action

          }
          break;
      }
    }
  }

  g_kb.unregisterExecutor(executor_id);
  feedback_pub.shutdown();
}

bool setGoal(urs_wearable::SetGoal::Request& req, urs_wearable::SetGoal::Response& res)
{
  std::thread thread_executor(executor, req);
  thread_executor.detach();
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

    g_action_client[i] = new (actionlib::SimpleActionClient<urs_wearable::ActionsAction>) ("/uav" + std::to_string(i) + "/action_server");
    g_action_client[i]->waitForServer();
  }

  // Set KB state publisher
  ros::Publisher state_pub = nh.advertise<urs_wearable::State>("/urs_wearable/state", 100);
  g_kb.setStatePub(&state_pub);

  // Set initial state
  std::vector<urs_wearable::Predicate> initial_state;
  urs_wearable::Predicate pred;
  pred.type = urs_wearable::Predicate::TYPE_TOOK_OFF;
  pred.predicate_took_off.truth_value = false;

  for (unsigned int i = 0; i < N_UAV; i++)
  {
    pred.predicate_took_off.drone_id.value = i;
    initial_state.push_back(pred);
  }
  g_kb.upsertPredicates(initial_state);

  // Advertise services
  ros::ServiceServer set_goal_service = nh.advertiseService("/urs_wearable/set_goal", setGoal);
  ros::ServiceServer location_add_service = nh.advertiseService("/urs_wearable/location_add", locationAdd);
  ros::ServiceServer location_remove_service = nh.advertiseService("/urs_wearable/location_remove", locationRemove);

  ROS_INFO("Waiting for connections from wearable devices...");

  ros::spin();

  // Clean up
  set_goal_service.shutdown();
  location_add_service.shutdown();
  location_remove_service.shutdown();

  for (int i = 0; i < N_UAV; i++)
  {
    delete g_action_client[i];
  }
}
