#include <mutex>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include "urs_wearable/navigator.h"
#include "urs_wearable/DroneAction.h"
#include "urs_wearable/PoseEuler.h"
#include "urs_wearable/SetDest.h"

class DroneActionServer
{
  actionlib::SimpleActionServer<urs_wearable::DroneAction> as_;
  std::string ns_;
  std::string name_;

  urs_wearable::DroneFeedback feedback_;
  urs_wearable::DroneResult result_;

  ros::Subscriber pose_sub_;
  urs_wearable::PoseEuler pose_;
  std::mutex pose_mutex_;

  double frequency_;

public:
  DroneActionServer(ros::NodeHandle& nh, const std::string& ns, const std::string& name) :
    as_(nh, name, boost::bind(&DroneActionServer::droneActionCb, this, _1), false), ns_(ns), name_(name)
  {
    frequency_ = 10.0;

    // Subscribe to the data topic of interest
    pose_sub_ = nh.subscribe(ns_ + "/urs_wearable/pose_euler", 10, &DroneActionServer::poseCb, this);

    as_.start();
    ROS_INFO("Server %s started", name_.c_str());
  }

  ~DroneActionServer()
  {
    pose_sub_.shutdown();
    ROS_INFO("Server %s destroyed", name_.c_str());
  }

  void poseCb(const urs_wearable::PoseEulerConstPtr& pose)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;

    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_ = *pose;
  }

  void droneActionCb(const urs_wearable::DroneGoalConstPtr &goal)
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
      ROS_INFO("%s: actionGoto set_dest OK", name_.c_str());

      ros::Rate r(frequency_);
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

        ros::spinOnce();
        r.sleep();
      }
    }
    else
    {
      ROS_ERROR("%s: actionGoto set_dest FAILED", name_.c_str());
    }

    as_.setAborted();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_action_server");
  ros::NodeHandle nh;
  std::string ns;

  if (!ros::param::get("~ns", ns))
  {
    ROS_ERROR("Failed to get a namespace for drone_action");
    return EXIT_FAILURE;
  }

  DroneActionServer drone_action_server(nh, ns, ns + "/action/drone");
  ros::spin();

  return 0;
}
