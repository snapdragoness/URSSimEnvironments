#include <mutex>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_srvs/Empty.h>
#include <urs_wearable/DroneAction.h>
#include <urs_wearable/SetPosition.h>

#include "urs_wearable/common.h"

class DroneActionServer
{
  actionlib::SimpleActionServer<urs_wearable::DroneAction> as_;
  std::string name_;

  urs_wearable::DroneFeedback feedback_;
  urs_wearable::DroneResult result_;

  ros::ServiceClient enable_motors_client_;

  ros::Subscriber pose_sub_;
  geometry_msgs::Pose pose_;
  std::mutex pose_mutex_;

  double dist_tolerance_;
  double frequency_;
  double landing_height_;
  double takeoff_height_;

public:
  DroneActionServer(ros::NodeHandle& nh, const std::string& name) :
    as_(nh, name, boost::bind(&DroneActionServer::droneActionCb, this, _1), false), name_(name)
  {
    dist_tolerance_ = 0.5;
    frequency_ = 10.0;
    landing_height_ = 0.3;
    takeoff_height_ = 2.0;

    enable_motors_client_ = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    pose_sub_ = nh.subscribe("ground_truth_to_tf/pose", 10, &DroneActionServer::poseCb, this);

    as_.start();
  }

  ~DroneActionServer()
  {
    pose_sub_.shutdown();
  }

  void poseCb(const geometry_msgs::PoseStampedConstPtr& pose_stamped)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_ = pose_stamped->pose;
  }

  void droneActionCb(const urs_wearable::DroneGoalConstPtr& goal)
  {
    switch (goal->action_type)
    {
      case urs_wearable::DroneGoal::TYPE_LANDING:
        actionLanding(goal);
        break;
      case urs_wearable::DroneGoal::TYPE_POSE:
        actionPose(goal);
        break;
      case urs_wearable::DroneGoal::TYPE_TAKEOFF:
        actionTakeoff(goal);
        break;
    }
  }

  void actionLanding(const urs_wearable::DroneGoalConstPtr& goal)
  {
    urs_wearable::SetPosition set_position_srv;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      set_position_srv.request.position = pose_.position;
    }
    set_position_srv.request.position.z = landing_height_;

    if (ros::service::call("set_position", set_position_srv))
    {
      ros::Rate rate(frequency_);
      while (ros::ok() && as_.isActive())
      {
        if (as_.isPreemptRequested())
        {
          if (!as_.isNewGoalAvailable())
          {
            std_srvs::Empty stop_srv;
            ros::service::call("stop", stop_srv);
          }

          as_.setPreempted();
          return;
        }

        {
          std::lock_guard<std::mutex> lock(pose_mutex_);

          if (pointDistance3D(pose_.position, set_position_srv.request.position) < dist_tolerance_)
          {
            break;
          }
        }

        ros::spinOnce();
        rate.sleep();
      }
    }
    else
    {
      ROS_WARN("%s: actionLanding called %s failed", name_.c_str(), ros::names::resolve("set_position").c_str());
      as_.setAborted();
      return;
    }

    // Disable motors
    hector_uav_msgs::EnableMotors enable_motors_srv;
    enable_motors_srv.request.enable = false;

    if (enable_motors_client_.call(enable_motors_srv) && enable_motors_srv.response.success)
    {
      as_.setSucceeded();
    }
    else
    {
      as_.setAborted();
    }
  }

  void actionPose(const urs_wearable::DroneGoalConstPtr& goal)
  {
    urs_wearable::SetPosition set_position_srv;
    set_position_srv.request.position = goal->pose.position;

    if (ros::service::call("set_position", set_position_srv))
    {
      ros::Rate rate(frequency_);
      while (ros::ok() && as_.isActive())
      {
        if (as_.isPreemptRequested())
        {
          if (!as_.isNewGoalAvailable())
          {
            std_srvs::Empty stop_srv;
            ros::service::call("stop", stop_srv);
          }

          as_.setPreempted();
          return;
        }

        {
          std::lock_guard<std::mutex> lock(pose_mutex_);

          if (pointDistance3D(pose_.position, set_position_srv.request.position) < dist_tolerance_)
          {
            as_.setSucceeded();
            return;
          }
        }

        ros::spinOnce();
        rate.sleep();
      }
    }
    else
    {
      ROS_WARN("%s: actionPose called %s failed", name_.c_str(), ros::names::resolve("set_position").c_str());
    }

    as_.setAborted();
  }

  void actionTakeoff(const urs_wearable::DroneGoalConstPtr& goal)
  {
    // Enable motors
    hector_uav_msgs::EnableMotors enable_motors_srv;
    enable_motors_srv.request.enable = true;

    if (!enable_motors_client_.call(enable_motors_srv) || !enable_motors_srv.response.success)
    {
      as_.setAborted();
      return;
    }

    urs_wearable::DroneGoal pose_goal;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose_goal.pose = pose_;
    }
    pose_goal.pose.position.z = takeoff_height_;

    actionPose(boost::make_shared<urs_wearable::DroneGoal>(pose_goal));
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_action");
  ros::NodeHandle nh;

  DroneActionServer drone_action_server(nh, "action/drone");

  ros::spin();

  return EXIT_SUCCESS;
}
