#include <thread>

#include <tf/transform_datatypes.h>

#include "urs_wearable/controller.h"

Controller::Controller()
{
  has_set_ns_ = false;

  pid_.p = 0.5;  // default: 0.5
  pid_.i = 0.0;
  pid_.d = 0.0;

  error_.position.x = error_.position.y = error_.position.z = error_.orientation.z = 0.0;
  integral_.position.x = integral_.position.y = integral_.position.z = integral_.orientation.z = 0.0;
}

Controller::~Controller()
{
  pose_sub_.shutdown();
  cmd_pub_.shutdown();
}

bool Controller::setNamespace(ros::NodeHandle& nh, const std::string& ns)
{
  if (!has_set_ns_)
  {
    ns_ = ns;
    has_set_ns_ = true;

    cmd_pub_ = nh.advertise<geometry_msgs::Twist>(ns_ + "/cmd_vel", 10, false);
    pose_sub_ = nh.subscribe(ns_ + "/ground_truth_to_tf/pose", 10, &Controller::controller, this);

    set_dest_service_ = nh.advertiseService(ns_ + "/set_dest", &Controller::setDest, this);

    std::thread pose_euler_publish_thread(&Controller::poseEulerPublish, this, std::ref(nh), 10);
    pose_euler_publish_thread.detach();

    return true;
  }

  return false;
}

void Controller::controller(const geometry_msgs::PoseStampedConstPtr& msg)
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
    yaw += 2 * M_PI;
  }

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose.position.x = msg->pose.position.x;
    pose.position.y = msg->pose.position.y;
    pose.position.z = msg->pose.position.z;
    pose.orientation.z = yaw;
  }

  prev_error_.position.x = error_.position.x;
  prev_error_.position.y = error_.position.y;
  prev_error_.position.z = error_.position.z;
  prev_error_.orientation.z = error_.orientation.z;

  {
    std::lock_guard<std::mutex> lock(dest_mutex_);
    error_.position.x = dest.position.x - msg->pose.position.x;
    error_.position.y = dest.position.y - msg->pose.position.y;
    error_.position.z = dest.position.z - msg->pose.position.z;
    error_.orientation.z = dest.orientation.z - yaw;
  }

  if (error_.orientation.z < -M_PI) {
    error_.orientation.z += M_PI + M_PI;
  } else if (error_.orientation.z > M_PI) {
    error_.orientation.z -= M_PI + M_PI;
  }

  proportional_.position.x = pid_.p * error_.position.x;
  proportional_.position.y = pid_.p * error_.position.y;
  proportional_.position.z = pid_.p * error_.position.z;
  integral_.position.x += pid_.i * error_.position.x;
  integral_.position.y += pid_.i * error_.position.y;
  integral_.position.z += pid_.i * error_.position.z;
  derivation_.position.x = pid_.d * (error_.position.x - prev_error_.position.x);
  derivation_.position.y = pid_.d * (error_.position.y - prev_error_.position.y);
  derivation_.position.z = pid_.d * (error_.position.z - prev_error_.position.z);

  double x, y, z;
  x = proportional_.position.x + integral_.position.x + derivation_.position.x;
  y = proportional_.position.y + integral_.position.y + derivation_.position.y;
  z = proportional_.position.z + integral_.position.z + derivation_.position.z;

  double rx, ry;
  rx = x * std::cos(-1 * yaw) - y * std::sin(-1 * yaw);
  ry = x * std::sin(-1 * yaw) + y * std::cos(-1 * yaw);

  cmd.linear.x = rx;
  cmd.linear.y = ry;
  cmd.linear.z = z;
  cmd.angular.z = 2.0 * error_.orientation.z;

  cmd_pub_.publish(cmd);
  ros::spinOnce();
}

void Controller::poseEulerPublish(ros::NodeHandle& nh, ros::Rate rate)
{
  ros::Publisher pose_pub = nh.advertise<urs_wearable::PoseEuler>(ns_ + "/urs_wearable/pose_euler", 10, false);

  while (ros::ok())
  {
    urs_wearable::PoseEuler pose;

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose.position.x = this->pose.position.x;
      pose.position.y = this->pose.position.y;
      pose.position.z = this->pose.position.z;
      pose.orientation.z = this->pose.orientation.z;
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
  urs_wearable::PoseEuler pose = this->pose;
  return pose;
}

urs_wearable::PoseEuler Controller::getDest()
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  urs_wearable::PoseEuler dest = this->dest;
  return dest;
}

void Controller::setDest(const urs_wearable::PoseEuler& dest, bool set_orientation)
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  this->dest.position.x = dest.position.x;
  this->dest.position.y = dest.position.y;
  this->dest.position.z = dest.position.z;
  if (set_orientation)
  {
    this->dest.orientation.z = dest.orientation.z;
  }
}

bool Controller::setDest(urs_wearable::SetDest::Request& req, urs_wearable::SetDest::Response& res)
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  this->dest.position.x = req.dest.position.x;
  this->dest.position.y = req.dest.position.y;
  this->dest.position.z = req.dest.position.z;
  if (req.set_orientation)
  {
    this->dest.orientation.z = req.dest.orientation.z;
  }
  return true;
}

double Controller::quaternionToYaw(const geometry_msgs::QuaternionConstPtr& q)
{
  double ysqr = q->y * q->y;

  double t3 = +2.0 * (q->w * q->z + q->x * q->y);
  double t4 = +1.0 - 2.0 * (ysqr + q->z * q->z);

  //  yaw (z-axis rotation)
  //          PI/2
  //            ^
  //            |
  //  PI,-PI <--+--> 0
  //            |
  //            v
  //         -PI/2
  double yaw = std::atan2(t3, t4);

  //  return the non-negative version
  //          PI/2
  //            ^
  //            |
  //      PI <--+--> 0
  //            |
  //            v
  //          3PI/2
  return (yaw < 0.0)? yaw + 2 * M_PI: yaw;
}

//////////////////////////////////////////////////////////////////////////////
/*
typedef struct Euler
{
  double roll;
  double pitch;
  double yaw;
} Euler;

Euler quaternionToEuler(const geometry_msgs::Quaternion::ConstPtr& q)
{
  Euler euler;

  double ysqr = q->y * q->y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (q->w * q->x + q->y * q->z);
  double t1 = +1.0 - 2.0 * (q->x * q->x + ysqr);
  euler.roll = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (q->w * q->y - q->z * q->x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  euler.pitch = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q->w * q->z + q->x * q->y);
  double t4 = +1.0 - 2.0 * (ysqr + q->z * q->z);
  euler.yaw = std::atan2(t3, t4);

  return euler;
}
*/
