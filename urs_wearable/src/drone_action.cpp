#include <atomic>
#include <future>
#include <list>
#include <mutex>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include "urs_wearable/common.h"
#include "urs_wearable/knowledge_base.h"

#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Empty.h>
#include <urs_wearable/AddDroneGoal.h>
#include <urs_wearable/DroneAction.h>
#include <urs_wearable/Feedback.h>
#include <urs_wearable/SetDroneActionStatus.h>
#include <urs_wearable/SetPosition.h>
#include <urs_wearable/RemoveDroneGoal.h>

class DroneActionServer
{
  const double MAX_POSITION_ERROR = 0.5;
  const double GROUND_HEIGHT = 0.182464;      // The height reported when the drone is on the ground
  const double SONAR_LANDING_HEIGHT = 0.3;
  const double TAKEOFF_HEIGHT = 15.0;

  actionlib::SimpleActionServer<urs_wearable::DroneAction> as_;
  std::string name_;

  actionlib::SimpleActionClient<urs_wearable::DroneAction> ac_;

  std::list<urs_wearable::DroneGoal> drone_goal_queue_;
  std::mutex drone_goal_queue_mutex_;

  ros::ServiceClient enable_motors_client_;
  ros::ServiceClient set_drone_action_status_client_;

  ros::ServiceServer add_drone_goal_service_;
  ros::ServiceServer clear_drone_goal_service_;
  ros::ServiceServer remove_drone_goal_service_;

  ros::Subscriber pose_sub_;
  geometry_msgs::Pose pose_;
  std::mutex pose_mutex_;

  ros::Subscriber sonar_height_sub_;
  std::atomic<float> sonar_height_;

  std::thread executor_thread_;
  std::promise<void> executor_thread_exit_signal_;

public:
  DroneActionServer(ros::NodeHandle& nh, const std::string& name) :
    as_(nh, name, boost::bind(&DroneActionServer::droneActionCb, this, _1), false), name_(name),
    ac_(nh, name, true),
    executor_thread_(&DroneActionServer::execute, this)
  {
    enable_motors_client_ = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    set_drone_action_status_client_ = nh.serviceClient<urs_wearable::SetDroneActionStatus>("/urs_wearable/set_drone_action_status");

    add_drone_goal_service_ = nh.advertiseService("add_drone_goal", &DroneActionServer::addDroneGoalService, this);
    clear_drone_goal_service_ = nh.advertiseService("clear_drone_goal", &DroneActionServer::clearDroneGoalService, this);
    remove_drone_goal_service_ = nh.advertiseService("remove_drone_goal", &DroneActionServer::removeDroneGoalService, this);

    pose_sub_ = nh.subscribe("ground_truth_to_tf/pose", 10, &DroneActionServer::poseCb, this);
    sonar_height_sub_ = nh.subscribe("sonar_height", 1, &DroneActionServer::sonarHeightCb, this);

    as_.start();
    ac_.waitForServer();
  }

  ~DroneActionServer()
  {
    executor_thread_exit_signal_.set_value();
    executor_thread_.join();
  }

  void execute()
  {
    std::future<void> future_obj = executor_thread_exit_signal_.get_future();

    while (future_obj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
      KnowledgeBase::executor_id_type executor_id;
      unsigned int action_index;
      bool has_goal = false;
      {
        std::lock_guard<std::mutex> lock(drone_goal_queue_mutex_);
        if (!drone_goal_queue_.empty())
        {
          ac_.sendGoal(drone_goal_queue_.front());

          executor_id = drone_goal_queue_.front().executor_id;
          action_index = drone_goal_queue_.front().action_index;
          has_goal = true;

          drone_goal_queue_.pop_front();
        }
      }

      if (has_goal)
      {
        // Tell the execution monitor what action it is executing
        urs_wearable::SetDroneActionStatus set_drone_action_status_srv;
        set_drone_action_status_srv.request.executor_id = executor_id;
        set_drone_action_status_srv.request.action_index = action_index;
        set_drone_action_status_srv.request.status = urs_wearable::Feedback::STATUS_ACTIVE;

        if (!set_drone_action_status_client_.call(set_drone_action_status_srv))
        {
          ros_error("Cannot call service " + ros::names::resolve("/urs_wearable/set_drone_action_status"));
        }

        // Wait for execution result
        ac_.waitForResult();

        // Return the result to the execution monitor
        switch (ac_.getState().state_)
        {
          case actionlib::SimpleClientGoalState::ABORTED:
            set_drone_action_status_srv.request.status = urs_wearable::Feedback::STATUS_ABORTED;
            break;

          case actionlib::SimpleClientGoalState::PREEMPTED:
            set_drone_action_status_srv.request.status = urs_wearable::Feedback::STATUS_PREEMPTED;
            break;

          case actionlib::SimpleClientGoalState::SUCCEEDED:
            set_drone_action_status_srv.request.status = urs_wearable::Feedback::STATUS_SUCCEEDED;
            break;

          default:
            set_drone_action_status_srv.request.status = urs_wearable::Feedback::STATUS_ABORTED;
            ros_error("Unexpected return in action result");
        }

        if (!set_drone_action_status_client_.call(set_drone_action_status_srv))
        {
          ros_error("Cannot call service " + ros::names::resolve("/urs_wearable/set_drone_action_status"));
        }
      }
      else
      {
        ros::Duration(0.1).sleep();
      }
    }
  }

  bool addDroneGoalService(urs_wearable::AddDroneGoal::Request& req, urs_wearable::AddDroneGoal::Response& res)
  {
    if (!req.drone_goals.empty())
    {
      // New drone goals will replace the previous ones
      if (as_.isActive())
      {
        ac_.cancelGoal();
      }

      std::lock_guard<std::mutex> lock(drone_goal_queue_mutex_);
      drone_goal_queue_.clear();
      drone_goal_queue_.insert(drone_goal_queue_.end(), req.drone_goals.begin(), req.drone_goals.end());

      return true;
    }

    return false;
  }

  bool clearDroneGoalService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if (as_.isActive())
    {
      ac_.cancelGoal();
    }

    std::lock_guard<std::mutex> lock(drone_goal_queue_mutex_);
    drone_goal_queue_.clear();

    return true;
  }

  bool removeDroneGoalService(urs_wearable::RemoveDroneGoal::Request& req, urs_wearable::RemoveDroneGoal::Response& res)
  {
    // Assumption: There can be only actions from one executor_id at a time
    std::lock_guard<std::mutex> lock(drone_goal_queue_mutex_);
    if (!drone_goal_queue_.empty()
        && drone_goal_queue_.front().executor_id == req.executor_id)
    {
      if (as_.isActive())
      {
        ac_.cancelGoal();
      }

      drone_goal_queue_.clear();
    }

    return true;
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
