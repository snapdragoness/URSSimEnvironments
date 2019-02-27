#include <atomic>
#include <mutex>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Empty.h>
#include <urs_wearable/DroneAction.h>
#include <urs_wearable/SetPosition.h>


#include "urs_wearable/common.h"

class DroneActionServer
{
  const double MAX_POSITION_ERROR = 0.5;
  const double GROUND_HEIGHT = 0.182464;      // The height reported when the drone is on the ground
  const double SONAR_LANDING_HEIGHT = 0.3;
  const double TAKEOFF_HEIGHT = 15.0;

  actionlib::SimpleActionServer<urs_wearable::DroneAction> as_;
  std::string name_;

  urs_wearable::DroneFeedback feedback_;
  urs_wearable::DroneResult result_;

  ros::ServiceClient enable_motors_client_;

  ros::Subscriber pose_sub_;
  geometry_msgs::Pose pose_;
  std::mutex pose_mutex_;

  ros::Subscriber sonar_height_sub_;
  std::atomic<float> sonar_height_;

//  ros::Subscriber sonar_upward_sub_;
//  std::atomic<float> sonar_upward_;

public:
  DroneActionServer(ros::NodeHandle& nh, const std::string& name) :
    as_(nh, name, boost::bind(&DroneActionServer::droneActionCb, this, _1), false), name_(name)
  {
    enable_motors_client_ = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    pose_sub_ = nh.subscribe("ground_truth_to_tf/pose", 10, &DroneActionServer::poseCb, this);
    sonar_height_sub_ = nh.subscribe("sonar_height", 1, &DroneActionServer::sonarHeightCb, this);
//    sonar_upward_sub_ = nh.subscribe("sonar_upward", 1, &DroneActionServer::sonarUpwardCb, this);

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

  void sonarHeightCb(const sensor_msgs::RangeConstPtr& msg)
  {
    sonar_height_ = msg->range;
  }

//  void sonarUpwardCb(const sensor_msgs::RangeConstPtr& msg)
//  {
//    sonar_upward_ = msg->range;
//  }

  void droneActionCb(const urs_wearable::DroneGoalConstPtr& goal)
  {
    switch (goal->action_type)
    {
      case urs_wearable::DroneGoal::TYPE_LAND:
        actionLand(goal);
        break;
      case urs_wearable::DroneGoal::TYPE_MOVE:
        actionMove(goal);
        break;
      case urs_wearable::DroneGoal::TYPE_TAKEOFF:
        actionTakeoff(goal);
        break;
    }
  }

  void actionLand(const urs_wearable::DroneGoalConstPtr& goal)
  {
    // Set position
    urs_wearable::SetPosition set_position_srv;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      set_position_srv.request.position = pose_.position;
    }
    set_position_srv.request.position.z = GROUND_HEIGHT;

    // Land
    if (ros::service::call("set_position", set_position_srv))
    {
      ros::Rate rate(10);
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

        if (sonar_height_ < SONAR_LANDING_HEIGHT)
        {
          std_srvs::Empty stop_srv;
          ros::service::call("stop", stop_srv);
          break;
        }

        ros::spinOnce();
        rate.sleep();
      }
    }
    else
    {
      ros_error("actionLanding called " + ros::names::resolve("enable_motors") + " failed");
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
      ros_error("actionLand called " + ros::names::resolve("set_position") + " failed");
      as_.setAborted();
    }
  }

  void actionMove(const urs_wearable::DroneGoalConstPtr& goal)
  {
    for (const auto& pose : goal->poses)
    {
      urs_wearable::SetPosition set_position_srv;
      set_position_srv.request.position = pose.position;

      if (ros::service::call("set_position", set_position_srv))
      {
        ros::Rate rate(10);
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

            if (pointDistance3D(pose_.position, set_position_srv.request.position) < MAX_POSITION_ERROR)
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
        ros_error("actionPose called " + ros::names::resolve("set_position") + " failed");
        as_.setAborted();
      }
    }

    as_.setSucceeded();
  }

  void actionTakeoff(const urs_wearable::DroneGoalConstPtr& goal)
  {
    // Enable motors
    hector_uav_msgs::EnableMotors enable_motors_srv;
    enable_motors_srv.request.enable = true;

    if (!enable_motors_client_.call(enable_motors_srv) || !enable_motors_srv.response.success)
    {
      ros_error("actionTakeoff called " + ros::names::resolve("enable_motors") + " failed");
      as_.setAborted();
      return;
    }

    // Set position
    urs_wearable::SetPosition set_position_srv;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      set_position_srv.request.position = pose_.position;
    }

    if (set_position_srv.request.position.z > TAKEOFF_HEIGHT - 2.0)
    {
      set_position_srv.request.position.z += 2.0;
    }
    else
    {
      set_position_srv.request.position.z = TAKEOFF_HEIGHT;
    }

    // Take off
    if (ros::service::call("set_position", set_position_srv))
    {
      ros::Rate rate(10);
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
          if (std::fabs(pose_.position.z - set_position_srv.request.position.z) < MAX_POSITION_ERROR)
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
      ros_error("actionTakeoff called " + ros::names::resolve("set_position") + " failed");
      as_.setAborted();
    }

    as_.setSucceeded();
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
