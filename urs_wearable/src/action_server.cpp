#include <chrono>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "urs_wearable/navigator.h"
#include "urs_wearable/ActionsAction.h"
#include "urs_wearable/PoseEuler.h"
#include "urs_wearable/SetDest.h"

class Actions
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  std::string ns;
  actionlib::SimpleActionServer<urs_wearable::ActionsAction> as;
  std::string action_name;
  urs_wearable::ActionsFeedback feedback;
  urs_wearable::ActionsResult result;

  ros::Subscriber pose_sub_;
  urs_wearable::PoseEuler pose_;
  std::mutex pose_mutex_;

public:
  Actions(std::string name) :
      as(nh, name, boost::bind(&Actions::executeCB, this, _1), false), action_name(name), private_nh("~")
  {
    if (private_nh.getParam("ns", ns))
    {
      ROS_INFO("%s: namespace = %s", action_name.c_str(), ns.c_str());
    }
    else
    {
      ROS_ERROR("%s: failed to get namespace", action_name.c_str());
    }

    // Register the goal and feeback callbacks
//    as.registerGoalCallback(boost::bind(&Actions::goalCB, this));
//    as.registerPreemptCallback(boost::bind(&Actions::preemptCB, this));

    // Subscribe to the data topic of interest
    pose_sub_ = nh.subscribe(ns + "/urs_wearable/pose_euler", 10, &Actions::poseCB, this);

    as.start();
    ROS_INFO("%s: started", action_name.c_str());
  }

  ~Actions()
  {
    pose_sub_.shutdown();

    ROS_INFO("%s: destroyed", action_name.c_str());
  }

  void poseCB(const urs_wearable::PoseEulerConstPtr& pose)
  {
    // make sure that the action hasn't been canceled
    if (!as.isActive())
      return;

    std::lock_guard<std::mutex> lock(pose_mutex_);
//    pose_ = pose;
//    pose_.position.x = pose->position.x;
//    pose_.position.y = pose->position.y;
//    pose_.position.z = pose->position.z;
//    pose_.orientation.z = pose->orientation.z;
  }

  void executeCB(const urs_wearable::ActionsGoalConstPtr &goal)
  {
    switch (goal->action_type)
    {
      case 0:
        actionGoto(goal);
        break;
    }
  }

  void actionGoto(const urs_wearable::ActionsGoalConstPtr &goal)
  {
    urs_wearable::SetDest set_dest_srv;
    set_dest_srv.request.dest.position.x = goal->pose.position.x;
    set_dest_srv.request.dest.position.y = goal->pose.position.y;
    set_dest_srv.request.dest.position.z = goal->pose.position.z;
    set_dest_srv.request.dest.orientation.z = goal->pose.orientation.z;

    if (ros::service::call(ns + "/set_dest", set_dest_srv))
    {
      ROS_INFO("%s: actionGoto set_dest OK", action_name.c_str());
    }
    else
    {
      ROS_INFO("%s: actionGoto set_dest FAILED", action_name.c_str());
    }

    // TODO: Monitor the ongoing action

//    urs_wearable::Pose dest;
//    dest.x = goal->pose.x;
//    dest.y = goal->pose.y;
//    dest.z = goal->pose.z;
//    while (ros::ok() && as.isActive())
//    {
//      urs_wearable::GetPose getPose;
//      if (!ros::service::call(ns + "/get_pose", getPose))
//      {
//        ROS_INFO("%s: actionGoto get_pose FAILED", action_name.c_str());
//        as.setAborted();
//        break;
//      }
//
//      urs_wearable::Pose pose;
//      pose.x = getPose.response.pose.x;
//      pose.y = getPose.response.pose.y;
//      pose.z = getPose.response.pose.z;
//
//      if (Navigator::getDistance(pose, dest) < 0.2)
//      {
//        ROS_INFO("%s: actionGoto destination REACHED", action_name.c_str());
//        as.setSucceeded();
//        break;
//      }
//
//      std::this_thread::sleep_for(std::chrono::milliseconds(10));
//    }

    as.setSucceeded();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");

  Actions actions(ros::this_node::getName());
  ros::spin();

  return 0;
}
