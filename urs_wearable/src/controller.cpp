#include <thread>

#include <tf/transform_datatypes.h>

#include "urs_wearable/common.h"
#include "urs_wearable/controller.h"

Controller::Controller()
{
  pid_ = pid_default_ ;

  error_.position.x = error_.position.y = error_.position.z = error_.orientation.z = 0.0;
  integral_.position.x = integral_.position.y = integral_.position.z = integral_.orientation.z = 0.0;

  geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("ground_truth_to_tf/pose");
  dest_.position = msg->pose.position;

  ros::NodeHandle nh;
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, false);

  pose_sub_ = nh.subscribe("ground_truth_to_tf/pose", 1, &Controller::pidControl, this);
  depth_image_sub_ = nh.subscribe("camera/depth/image_raw", 1, &Controller::readDepthImage, this);
  sonar_height_sub_ = nh.subscribe<sensor_msgs::Range>("sonar_height", 1, &Controller::readSonarHeight, this);
  sonar_upward_sub_ = nh.subscribe<sensor_msgs::Range>("sonar_upward", 1, &Controller::readSonarUpward, this);

  get_dest_service_ = nh.advertiseService("get_dest", &Controller::getDestService, this);
  get_pose_service_ = nh.advertiseService("get_pose", &Controller::getPoseService, this);
  set_dest_service_ = nh.advertiseService("set_dest", &Controller::setDest, this);
  set_altitude_service_ = nh.advertiseService("set_altitude", &Controller::setAltitude, this);

  std::thread pose_euler_publish_thread(&Controller::poseEulerPublish, this);
  pose_euler_publish_thread.detach();
}

Controller::~Controller()
{
  cmd_pub_.shutdown();
  depth_image_sub_.shutdown();
  pose_sub_.shutdown();
  sonar_height_sub_.shutdown();
  sonar_upward_sub_.shutdown();

  get_dest_service_.shutdown();
  set_dest_service_.shutdown();
  set_altitude_service_.shutdown();
}

void Controller::readDepthImage(const sensor_msgs::Image::ConstPtr& msg)
{
  /* https://answers.ros.org/question/246066/how-can-i-get-object-distance-using-cameradepthimage_raw
   * depth_array[0], depth_array[1], ..., depth_aray[image.height * image.width - 1] */
  if (is_moving_ && msg->encoding.compare("32FC1") == 0)
  {
    const float* depth_image_array = reinterpret_cast<const float*>(&(msg->data[0]));

    if (getPose().position.z > 1.0)
    {
      for (int i = 225; i < 255; i++)
      {
        for (int j = 280; j < 360; j++)
        {
          int index = i * 640 + j;
          float depth = depth_image_array[i * 640 + j];

          if (!std::isnan(depth) &&
              !std::isnan(depth_image_array[index - 1]) &&
              !std::isnan(depth_image_array[index + 1]) &&
              !std::isnan(depth_image_array[index - 640]) &&
              !std::isnan(depth_image_array[index + 640]))
          {
            if (pointDistance2D(getPose().position, getDest().position) > depth)
            {
              avoidObstacle();
              return;
            }
          }
        }
      }
    }
  }
}

void Controller::avoidObstacle()
{
  ros_warn("Avoiding obstacle");

  urs_wearable::PoseEuler original_dest = this->getDest();

  stop();

  urs_wearable::PoseEuler pose = this->getPose();
  pose.position.z += 2.0;
  this->setDest(pose, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  this->setDest(original_dest, false);
}

void Controller::pidControl(const geometry_msgs::PoseStampedConstPtr& msg)
{
  /* https://answers.ros.org/question/11545/plotprint-rpy-from-quaternion */
  // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);

  // The tf::Quaternion has a method to access roll, pitch, and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // This line is subject to change according to Controller::quaternionToYaw(const geometry_msgs::QuaternionConstPtr&);
  if (yaw < 0.0)
  {
    yaw += M_PI + M_PI;
  }

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_.position.x = msg->pose.position.x;
    pose_.position.y = msg->pose.position.y;
    pose_.position.z = msg->pose.position.z;
    pose_.orientation.z = yaw;
  }

  prev_error_.position.x = error_.position.x;
  prev_error_.position.y = error_.position.y;
  prev_error_.position.z = error_.position.z;
  prev_error_.orientation.z = error_.orientation.z;

  {
    std::lock_guard<std::mutex> lock(dest_mutex_);
    error_.position.x = dest_.position.x - msg->pose.position.x;
    error_.position.y = dest_.position.y - msg->pose.position.y;
    error_.position.z = dest_.position.z - msg->pose.position.z;
    error_.orientation.z = dest_.orientation.z - yaw;

    if (pointDistance3D(msg->pose.position, dest_.position) < max_position_error_)
    {
      is_moving_ = false;
      setSpeed(pid_default_);
    }
  }

  if (error_.orientation.z < -M_PI) {
    error_.orientation.z += M_PI + M_PI;
  } else if (error_.orientation.z > M_PI) {
    error_.orientation.z -= M_PI + M_PI;
  }

  {
    std::lock_guard<std::mutex> lock(pid_mutex_);
    proportional_.position.x = pid_.p * error_.position.x;
    proportional_.position.y = pid_.p * error_.position.y;
    proportional_.position.z = pid_.p * error_.position.z;
    integral_.position.x += pid_.i * error_.position.x;
    integral_.position.y += pid_.i * error_.position.y;
    integral_.position.z += pid_.i * error_.position.z;
    derivation_.position.x = pid_.d * (error_.position.x - prev_error_.position.x);
    derivation_.position.y = pid_.d * (error_.position.y - prev_error_.position.y);
    derivation_.position.z = pid_.d * (error_.position.z - prev_error_.position.z);
  }

  double x, y, z;
  x = proportional_.position.x + integral_.position.x + derivation_.position.x;
  y = proportional_.position.y + integral_.position.y + derivation_.position.y;
  z = proportional_.position.z + integral_.position.z + derivation_.position.z;

  double rx, ry;
  rx = x * std::cos(-1 * yaw) - y * std::sin(-1 * yaw);
  ry = x * std::sin(-1 * yaw) + y * std::cos(-1 * yaw);

  geometry_msgs::Twist cmd;
  cmd.linear.x = rx;
  cmd.linear.y = ry;
  cmd.linear.z = z;
  cmd.angular.z = error_.orientation.z;

  cmd_pub_.publish(cmd);
}

void Controller::poseEulerPublish()
{
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<urs_wearable::PoseEuler>("urs_wearable/pose_euler", 1, false);

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    urs_wearable::PoseEuler pose;

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose.position.x = this->pose_.position.x;
      pose.position.y = this->pose_.position.y;
      pose.position.z = this->pose_.position.z;
      pose.orientation.z = this->pose_.orientation.z;
    }

    pose_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  /* clean up the publisher */
  pose_pub.shutdown();
}

urs_wearable::PoseEuler Controller::getPose()
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  urs_wearable::PoseEuler pose = this->pose_;
  return pose;
}

urs_wearable::PoseEuler Controller::getDest()
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  urs_wearable::PoseEuler dest = this->dest_;
  return dest;
}

void Controller::setDest(const urs_wearable::PoseEuler& dest, bool set_orientation)
{
  urs_wearable::PoseEuler pose = this->getPose();
  double vx = dest.position.x - pose.position.x;
  double vy = dest.position.y - pose.position.y;
  double yaw_to_dest = std::atan2(vy, vx);
  if (yaw_to_dest < 0.0)
  {
    yaw_to_dest += M_PI + M_PI;
  }

  double planar_dist_to_dest = std::sqrt(vx * vx + vy * vy);
  if (planar_dist_to_dest > max_position_error_)
  {
    {
      std::lock_guard<std::mutex> lock(dest_mutex_);
      dest_.orientation.z = yaw_to_dest;
    }

    do
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (yawDiff(this->getPose().orientation.z, yaw_to_dest) > max_orientation_error_ * M_PI / 180.0);
  }

  {
    std::lock_guard<std::mutex> lock(dest_mutex_);
    dest_.position.x = dest.position.x;
    dest_.position.y = dest.position.y;
    dest_.position.z = dest.position.z;
    is_moving_ = true;
  }
}

void Controller::setAltitude(double z)
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  this->dest_.position.z = z;
}

bool Controller::getPoseService(urs_wearable::GetPose::Request& req, urs_wearable::GetPose::Response& res)
{
  res.pose = getPose();
  return true;
}

bool Controller::getDestService(urs_wearable::GetDest::Request& req, urs_wearable::GetDest::Response& res)
{
  res.dest = getDest();
  return true;
}

bool Controller::setDest(urs_wearable::SetDest::Request& req, urs_wearable::SetDest::Response& res)
{
  setDest(req.dest, req.set_orientation);
  return true;
}

bool Controller::setAltitude(urs_wearable::SetAltitude::Request& req, urs_wearable::SetAltitude::Response& res)
{
  setAltitude(req.z);
  return true;
}

void Controller::stop()
{
  urs_wearable::PoseEuler pose = getPose();
  setDest(pose, false);
  ros::Duration(1.0).sleep(); // Sleep for 1 second
}

void Controller::setSpeed(const PID& pid)
{
  std::lock_guard<std::mutex> lock(pid_mutex_);
  pid_ = pid;
}

void Controller::readSonarHeight(const sensor_msgs::RangeConstPtr& msg)
{
  sonar_height_range_ = msg->range;
  // TODO: Check for collision
}

void Controller::readSonarUpward(const sensor_msgs::RangeConstPtr& msg)
{
  sonar_upward_range_ = msg->range;
  // TODO: Check for collision
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  // Make sure internal controller of hector_quadrotor is running
  if (!ros::service::waitForService("controller/attitude/yawrate/set_parameters", 60000) ||
      !ros::service::waitForService("controller/position/z/set_parameters", 60000) ||
      !ros::service::waitForService("controller/velocity/z/set_parameters", 60000))
  {
    ros_error("The drone has not been spawned");
  }

  Controller controller;

  ros::spin();

  return EXIT_SUCCESS;
}
