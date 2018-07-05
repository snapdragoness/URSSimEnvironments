#include <chrono>
#include <mutex>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include "urs_wearable/navigator.h"
#include "urs_wearable/DroneAction.h"
#include "urs_wearable/PoseEuler.h"
#include "urs_wearable/SetDest.h"

class Actions
{
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string ns_;

  actionlib::SimpleActionServer<urs_wearable::DroneAction> as_;
  std::string as_name_;
  urs_wearable::DroneFeedback feedback_;
  urs_wearable::DroneResult result_;

  ros::Subscriber pose_sub_;
  urs_wearable::PoseEuler pose_;
  std::mutex pose_mutex_;

public:
  Actions(const std::string& name) :
    as_(nh_, name, boost::bind(&Actions::executeCb, this, _1), false), as_name_(name), private_nh_("~")
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
    pose_sub_ = nh_.subscribe(ns_ + "/urs_wearable/pose_euler", 10, &Actions::poseCb, this);

    as_.start();
    ROS_INFO("%s: started", as_name_.c_str());
  }

  ~Actions()
  {
    pose_sub_.shutdown();
    ROS_INFO("%s: destroyed", as_name_.c_str());
  }

  void poseCb(const urs_wearable::PoseEulerConstPtr& pose)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;

    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_ = *pose;
  }

  void executeCb(const urs_wearable::DroneGoalConstPtr &goal)
  {
    switch (goal->action_type)
    {
      case urs_wearable::DroneGoal::TYPE_GOTO:
        actionGoto(goal);
        break;
    }
  }

  void actionGoto(const urs_wearable::DroneGoalConstPtr &goal)
  {
    urs_wearable::SetDest set_dest_srv;
    set_dest_srv.request.dest = goal->pose;
    set_dest_srv.request.set_orientation = goal->set_orientation;

    if (ros::service::call(ns_ + "/set_dest", set_dest_srv))
    {
      ROS_INFO("%s: actionGoto set_dest OK", as_name_.c_str());

      while (ros::ok() && as_.isActive())
      {
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);

          if (Navigator::getDistance(pose_, goal->pose) < 0.5)
          {
            as_.setSucceeded();
            return;
          }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    else
    {
      ROS_ERROR("%s: actionGoto set_dest FAILED", as_name_.c_str());
    }

    as_.setAborted();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");

  Actions actions(ros::this_node::getName());
  ros::spin();

  return 0;
}
