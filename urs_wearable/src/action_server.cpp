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
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string ns_;

  actionlib::SimpleActionServer<urs_wearable::ActionsAction> as_;
  std::string as_name_;
  urs_wearable::ActionsFeedback feedback_;
  urs_wearable::ActionsResult result_;

  ros::Subscriber pose_sub_;
  urs_wearable::PoseEuler pose_;
  std::mutex pose_mutex_;

public:
  Actions(const std::string& name) :
    as_(nh_, name, boost::bind(&Actions::executeCallback, this, _1), false), as_name_(name), private_nh_("~")
  {
    if (private_nh_.getParam("ns", ns_))
    {
      ROS_INFO("%s: namespace = %s", as_name_.c_str(), ns_.c_str());
    }
    else
    {
      ROS_ERROR("%s: failed to get namespace", as_name_.c_str());
    }

//    // Register the goal and feeback callbacks
//    as.registerGoalCallback(boost::bind(&Actions::goalCB, this));
//    as.registerPreemptCallback(boost::bind(&Actions::preemptCB, this));

    // Subscribe to the data topic of interest
    pose_sub_ = nh_.subscribe(ns_ + "/urs_wearable/pose_euler", 10, &Actions::poseCallback, this);

    as_.start();
    ROS_INFO("%s: started", as_name_.c_str());
  }

  ~Actions()
  {
    pose_sub_.shutdown();
    ROS_INFO("%s: destroyed", as_name_.c_str());
  }

  void poseCallback(const urs_wearable::PoseEulerConstPtr& pose)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;

    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_ = *pose;
  }

  void executeCallback(const urs_wearable::ActionsGoalConstPtr &goal)
  {
    switch (goal->action_type)
    {
      case urs_wearable::ActionsGoal::TYPE_GOTO:
        actionGoto(goal);
        break;
    }
  }

  void actionGoto(const urs_wearable::ActionsGoalConstPtr &goal)
  {
    urs_wearable::SetDest set_dest_srv;
    set_dest_srv.request.dest = goal->pose;
    set_dest_srv.request.set_orientation = goal->set_orientation;

    if (ros::service::call(ns_ + "/set_dest", set_dest_srv))
    {
      ROS_INFO("%s: actionGoto set_dest OK", as_name_.c_str());
    }
    else
    {
      ROS_INFO("%s: actionGoto set_dest FAILED", as_name_.c_str());
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

    as_.setSucceeded();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");

  Actions actions(ros::this_node::getName());
  ros::spin();

  return 0;
}
