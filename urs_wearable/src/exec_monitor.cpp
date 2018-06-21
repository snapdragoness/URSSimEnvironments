#include "urs_wearable/controller.h"
#include "urs_wearable/navigator.h"
#include "urs_wearable/pool.h"

#include "urs_wearable/PoseEuler.h"
#include "urs_wearable/Region.h"

#include "urs_wearable/GetPlan.h"
#include "urs_wearable/SetActiveRegion.h"
#include "urs_wearable/SetDest.h"

#include "urs_wearable/ActionsAction.h"

#include <ros/ros.h>
#include <actionlib/client/service_client.h>

#include <chrono>
#include <thread>

const unsigned int N_UAV = 4;
const unsigned int WP_POOL_SIZE = 100;
const std::string PLANNER_SERVICE_NAME = "/cpa/get_plan";

urs_wearable::Region activeRegion;
Pool<urs_wearable::DestEuler, WP_POOL_SIZE> wp_pool;

Controller controller[N_UAV];
Navigator navigator[N_UAV];
actionlib::SimpleActionClient<urs_wearable::ActionsAction>* action_client[N_UAV];

void initPlanningRequest(std::vector<int>&, urs_wearable::GetPlan::Request&);
void requestPlanAndExecute(urs_wearable::GetPlan&);

bool setDest(urs_wearable::SetDest::Request &req, urs_wearable::SetDest::Response &res)
{
  std::vector<int> allocatedWpList;

  // Construct planning request
  urs_wearable::GetPlan getPlanSrv;
  initPlanningRequest(allocatedWpList, getPlanSrv.request);

  for (int i = 0; i < req.dest.size(); i++)
  {
    int wpId = wp_pool.newId(allocatedWpList);
    while (wpId == -1)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      wpId = wp_pool.newId(allocatedWpList);
    }

    // Check that the requested dest is within active region
    if (req.dest[i].pose.position.x >= activeRegion.x0
        && req.dest[i].pose.position.x <= activeRegion.x1
        && req.dest[i].pose.position.y >= activeRegion.y0
        && req.dest[i].pose.position.y <= activeRegion.y1
        && req.dest[i].pose.position.z <= activeRegion.z)
    {
      wp_pool.data[wpId].pose.position.x = req.dest[i].pose.position.x;
      wp_pool.data[wpId].pose.position.y = req.dest[i].pose.position.y;
      wp_pool.data[wpId].pose.position.z = req.dest[i].pose.position.z;
      wp_pool.data[wpId].pose.orientation.z = req.dest[i].pose.orientation.z;
      wp_pool.data[wpId].set_orientation = req.dest[i].set_orientation;

      urs_wearable::PredicateAt predicateAt;
      predicateAt.uav_id = req.uav_id[i];
      predicateAt.wp_id = wpId;
      getPlanSrv.request.goal_state.at.push_back(predicateAt);
    }
    else
    {
      return false;
    }
  }

  requestPlanAndExecute(getPlanSrv);
  wp_pool.retrieveId(allocatedWpList);

  return true;
}

bool setActiveRegion(urs_wearable::SetActiveRegion::Request &req, urs_wearable::SetActiveRegion::Response &res)
{
  activeRegion.x0 = req.active_region.x0;
  activeRegion.x1 = req.active_region.x1;
  activeRegion.y0 = req.active_region.y0;
  activeRegion.y1 = req.active_region.y1;
  activeRegion.z = req.active_region.z;

  return true;
}

void initPlanningRequest(std::vector<int>& allocatedWpList, urs_wearable::GetPlan::Request& request)
{
  for (unsigned int i = 0; i < N_UAV; i++)
  {
    int wpId = wp_pool.newId(allocatedWpList);
    while (wpId == -1)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      wpId = wp_pool.newId(allocatedWpList);
    }

    wp_pool.data[wpId].pose = controller[i].getPose();
    wp_pool.data[wpId].set_orientation = false;

    urs_wearable::PredicateAt predicateAt;
    predicateAt.uav_id = i;
    predicateAt.wp_id = wpId;
    request.initial_state.at.push_back(predicateAt);
  }
}

void requestPlanAndExecute(urs_wearable::GetPlan& getPlanSrv)
{
  if (ros::service::call(PLANNER_SERVICE_NAME, getPlanSrv))
  {
    ROS_INFO("Call %s successfully", PLANNER_SERVICE_NAME.c_str());

    std::vector<urs_wearable::Action>& actions = getPlanSrv.response.actions;
    for (int i = 0; i < actions.size(); i++)
    {
      switch (actions[i].action_type)
      {
        case 0: // goto
        {
          int wpId = actions[i].action_goto.wp_id;
          urs_wearable::ActionsGoal goal;

          goal.action_type = 0;
          goal.pose.position.x = wp_pool.data[wpId].pose.position.x;
          goal.pose.position.y = wp_pool.data[wpId].pose.position.y;
          goal.pose.position.z = wp_pool.data[wpId].pose.position.z;
          goal.pose.orientation.z = wp_pool.data[wpId].pose.orientation.z;
          goal.set_orientation = wp_pool.data[wpId].set_orientation;

          actionlib::SimpleActionClient<urs_wearable::ActionsAction>& ac = *action_client[actions[i].action_goto.uav_id];

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

          break;
        }
      }
    }
  }
  else
  {
    ROS_INFO("Call %s failed", PLANNER_SERVICE_NAME.c_str());
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "exec_monitor");
  ros::NodeHandle nh;

  if (N_UAV == 0)
  {
    ROS_ERROR("No UAV to control");
    exit(EXIT_FAILURE);
  }

  // Set active region
  activeRegion.x0 = -50;
  activeRegion.y0 = -50;
  activeRegion.x1 = 50;
  activeRegion.y1 = 50;
  activeRegion.z = 20;

  // Start controllers, navigators, and action clients
  for (unsigned int i = 0; i < N_UAV; i++)
  {
    controller[i].init();
    controller[i].setNamespace("/uav" + std::to_string(i));
    controller[i].start();

    navigator[i].init();
    navigator[i].setNamespace("/uav" + std::to_string(i));

    action_client[i] = new (actionlib::SimpleActionClient<urs_wearable::ActionsAction>) ("/uav" + std::to_string(i) + "/action_server");
    action_client[i]->waitForServer();
  }

  // Advertise services
  ros::ServiceServer set_dest_service = nh.advertiseService("/urs_wearable/set_dest", setDest);
  ros::ServiceServer set_active_region_service = nh.advertiseService("/urs_wearable/set_active_region", setActiveRegion);

  ROS_INFO("Waiting for connections from wearable devices...");

  ros::spin();

  // Clean up
  set_dest_service.shutdown();

  for (int i = 0; i < N_UAV; i++)
  {
    delete action_client[i];
  }
}
