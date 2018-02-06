#include "urs_wearable/ActionsAction.h"
#include "urs_wearable/Pose.h"
#include "urs_wearable/SetDest.h"

#include "urs_wearable/navigator.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>

class Actions
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  std::string ns;
  actionlib::SimpleActionServer<urs_wearable::ActionsAction> as;
  std::string action_name;
  urs_wearable::ActionsFeedback feedback;
  urs_wearable::ActionsResult result;

  ros::Subscriber poseSub;
  Pose pose;
  boost::mutex mut_pose;

public:
  Actions(std::string name) :
      as(nh, name, boost::bind(&Actions::executeCB, this, _1), false), action_name(name), private_nh("~")
  {
    if (private_nh.getParam("ns", ns))
    {
      ROS_INFO("%s: ns = %s", action_name.c_str(), ns.c_str());
    }
    else
    {
      ROS_ERROR("%s: failed to get ns", action_name.c_str());
    }

    // Register the goal and feeback callbacks
//    as.registerGoalCallback(boost::bind(&Actions::goalCB, this));
//    as.registerPreemptCallback(boost::bind(&Actions::preemptCB, this));

    // Subscribe to the data topic of interest
    poseSub = nh.subscribe(ns + "/urs_wearable/pose", 10, &Actions::poseCB, this);

    as.start();
    ROS_INFO("%s: started", action_name.c_str());
  }

  ~Actions()
  {
    poseSub.shutdown();

    ROS_INFO("%s: destroyed", action_name.c_str());
  }

  void poseCB(const urs_wearable::PoseConstPtr& pose)
  {
    // make sure that the action hasn't been canceled
    if (!as.isActive())
      return;

    mut_pose.lock();
    this->pose.x = pose->x;
    this->pose.y = pose->y;
    this->pose.z = pose->z;
    this->pose.yaw = pose->yaw;
    mut_pose.unlock();
  }

  void executeCB(const urs_wearable::ActionsGoalConstPtr &goal)
  {
    switch (goal->action_type)
    {
      case 1:
        actionGoto(goal);
        break;
    }
  }

  void actionGoto(const urs_wearable::ActionsGoalConstPtr &goal)
  {
    urs_wearable::SetDest setDest;
    setDest.request.pose.x = goal->pose.x;
    setDest.request.pose.y = goal->pose.y;
    setDest.request.pose.z = goal->pose.z;
    setDest.request.rotate = goal->rotate;

    if (ros::service::call(ns + "/set_dest", setDest))
    {
      ROS_INFO("%s: actionGoto OK", action_name.c_str());
    }
    else
    {
      ROS_INFO("%s: actionGoto FAILED", action_name.c_str());
    }

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
