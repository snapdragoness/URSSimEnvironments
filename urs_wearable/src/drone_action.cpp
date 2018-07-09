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

  ros::ServiceClient enable_motors_client_;

  ros::Subscriber pose_sub_;
  urs_wearable::PoseEuler pose_;
  std::mutex pose_mutex_;

  double dist_tolerance_;
  double frequency_;
  double landing_height_;
  double takeoff_height_;

public:
  DroneActionServer(ros::NodeHandle& nh, const std::string& ns, const std::string& name) :
    as_(nh, name, boost::bind(&DroneActionServer::droneActionCb, this, _1), false), ns_(ns), name_(name)
  {
    dist_tolerance_ = 0.1;
    frequency_ = 10.0;
    landing_height_ = 0.3;
    takeoff_height_ = 2.0;

    enable_motors_client_ = nh.serviceClient<hector_uav_msgs::EnableMotors>(ns_ + "/enable_motors");
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
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_ = *pose;
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
    urs_wearable::SetDest set_dest_srv;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      set_dest_srv.request.dest = pose_;
    }
    set_dest_srv.request.dest.position.z = landing_height_;
    set_dest_srv.request.set_orientation = false;

    if (ros::service::call(ns_ + "/set_dest", set_dest_srv))
    {
      ros::Rate rate(frequency_);
      while (ros::ok() && as_.isActive())
      {
        if (as_.isPreemptRequested())
        {
          if (!as_.isNewGoalAvailable())
          {
            // Stop moving
            {
              std::lock_guard<std::mutex> lock(pose_mutex_);
              set_dest_srv.request.dest = pose_;
            }
            set_dest_srv.request.set_orientation = false;
            ros::service::call(ns_ + "/set_dest", set_dest_srv);
          }

          as_.setPreempted();
          return;
        }

        {
          std::lock_guard<std::mutex> lock(pose_mutex_);

          if (Navigator::getDistance(pose_, set_dest_srv.request.dest) < dist_tolerance_)
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
      ROS_WARN("%s: actionLanding called set_dest failed", name_.c_str());
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
    urs_wearable::SetDest set_dest_srv;
    set_dest_srv.request.dest = goal->pose;
    set_dest_srv.request.set_orientation = goal->set_orientation;

    if (ros::service::call(ns_ + "/set_dest", set_dest_srv))
    {
      ros::Rate rate(frequency_);
      while (ros::ok() && as_.isActive())
      {
        if (as_.isPreemptRequested())
        {
          if (!as_.isNewGoalAvailable())
          {
            // Stop moving
            {
              std::lock_guard<std::mutex> lock(pose_mutex_);
              set_dest_srv.request.dest = pose_;
            }
            set_dest_srv.request.set_orientation = false;
            ros::service::call(ns_ + "/set_dest", set_dest_srv);
          }

          as_.setPreempted();
          return;
        }

        {
          std::lock_guard<std::mutex> lock(pose_mutex_);

          if (Navigator::getDistance(pose_, set_dest_srv.request.dest) < dist_tolerance_)
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
      ROS_WARN("%s: actionPose called set_dest failed", name_.c_str());
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
    pose_goal.set_orientation = false;

    actionPose(boost::make_shared<urs_wearable::DroneGoal>(pose_goal));
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
