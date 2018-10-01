#include <thread>

#include <tf/transform_datatypes.h>

#include "urs_wearable/controller.h"

Controller::Controller()
{
  has_set_ns_ = false;

  pid_ = pid_default_ = {0.2, 0.0, 0.0};
  pid_slow_ = {0.1, 0.0, 0.0};

  error_.position.x = error_.position.y = error_.position.z = error_.orientation.z = 0.0;
  integral_.position.x = integral_.position.y = integral_.position.z = integral_.orientation.z = 0.0;

  depth_image_array_ = NULL;
}

Controller::~Controller()
{
  pose_sub_.shutdown();
  cmd_pub_.shutdown();
  depth_image_sub_.shutdown();
  sonar_height_sub_.shutdown();
  sonar_upward_sub_.shutdown();

  set_dest_service_.shutdown();
  set_altitude_service_.shutdown();
}

bool Controller::setNamespace(ros::NodeHandle& nh, const std::string& ns)
{
  if (!has_set_ns_)
  {
    ns_ = ns;
    has_set_ns_ = true;

    cmd_pub_ = nh.advertise<geometry_msgs::Twist>(ns_ + "/cmd_vel", 10, false);
    pose_sub_ = nh.subscribe(ns_ + "/ground_truth_to_tf/pose", 10, &Controller::controller, this);
    depth_image_sub_ = nh.subscribe(ns + "/camera/depth/image_raw", 1, &Controller::readDepthImage, this);
    sonar_height_sub_ = nh.subscribe<sensor_msgs::Range>(ns + "/sonar_height", 1, &Controller::readSonarHeight, this);
    sonar_upward_sub_ = nh.subscribe<sensor_msgs::Range>(ns + "/sonar_upward", 1, &Controller::readSonarUpward, this);

    set_dest_service_ = nh.advertiseService(ns_ + "/set_dest", &Controller::setDest, this);
    set_altitude_service_ = nh.advertiseService(ns_ + "/set_altitude", &Controller::setAltitude, this);

    std::thread pose_euler_publish_thread(&Controller::poseEulerPublish, this, std::ref(nh), 10);
    pose_euler_publish_thread.detach();

    geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(ns_ + "/ground_truth_to_tf/pose");
    std::lock_guard<std::mutex> lock(dest_mutex_);
    dest_.position = msg->pose.position;

    return true;
  }

  return false;
}

void Controller::readDepthImage(const sensor_msgs::Image::ConstPtr& msg)
{
  if (is_moving_ && msg->encoding.compare("32FC1") == 0)
  {
    /* https://answers.ros.org/question/246066/how-can-i-get-object-distance-using-cameradepthimage_raw
     * depth_array[0], depth_array[1], ..., depth_aray[image.height * image.width - 1] */
//    mut_depthImage.lock();
//    const float* depthImageArray = reinterpret_cast<const float*>(&(msg->data[0]));

    std::lock_guard<std::mutex> lock(depth_image_array_mutex_);
    depth_image_array_ = reinterpret_cast<const float*>(&(msg->data[0]));

    if (getPose().position.z > 1.0)
    {
      for (int i = 225; i < 255; i++)
      {
        for (int j = 280; j < 360; j++)
        {
          int index = i * 640 + j;
          float depth = depth_image_array_[i * 640 + j];

          if (!std::isnan(depth) &&
              !std::isnan(depth_image_array_[index - 1]) &&
              !std::isnan(depth_image_array_[index + 1]) &&
              !std::isnan(depth_image_array_[index - 640]) &&
              !std::isnan(depth_image_array_[index + 640]))
          {
            urs_wearable::PoseEuler pose = getPose();
            urs_wearable::PoseEuler dest = getDest();

            if (std::sqrt((pose.position.x - dest.position.x) * (pose.position.x - dest.position.x)
                          + (pose.position.y - dest.position.y) * (pose.position.y - dest.position.y)) > depth)
            {
              avoidObstacle();
              return;
            }
          }
        }
      }
    }

    // TODO: check obstacle
  }
}

void Controller::avoidObstacle()
{
  ROS_WARN("%s: avoiding obstacle", ros::this_node::getName().c_str());

  urs_wearable::PoseEuler original_dest = this->getDest();

  stop();

  urs_wearable::PoseEuler pose = this->getPose();
  pose.position.z += 2.0;
  this->setDest(pose, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  this->setDest(original_dest, false);
}

void Controller::printImage()
{
  std::lock_guard<std::mutex> lock(depth_image_array_mutex_);
  for (int i = 0; i < 480; i++)
  {
    for (int j = 0; j < 640; j++)
    {
      std::cout << depth_image_array_[640 * i + j] << " ";
    }
    std::cout << std::endl;
  }
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

    if (std::sqrt(
      (msg->pose.position.x - dest_.position.x) * (msg->pose.position.x - dest_.position.x)
      + (msg->pose.position.y - dest_.position.y) * (msg->pose.position.y - dest_.position.y)
      + (msg->pose.position.z - dest_.position.z) * (msg->pose.position.z - dest_.position.z)) < max_position_error_)
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

  cmd.linear.x = rx;
  cmd.linear.y = ry;
  cmd.linear.z = z;
  cmd.angular.z = error_.orientation.z;

  cmd_pub_.publish(cmd);
}

void Controller::poseEulerPublish(ros::NodeHandle& nh, ros::Rate rate)
{
  ros::Publisher pose_pub = nh.advertise<urs_wearable::PoseEuler>(ns_ + "/urs_wearable/pose_euler", 10, false);

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
  std::lock_guard<std::mutex> lock(dest_mutex_);
  this->dest_.position.x = dest.position.x;
  this->dest_.position.y = dest.position.y;
  this->dest_.position.z = dest.position.z;
  if (set_orientation)
  {
    this->dest_.orientation.z = dest.orientation.z;
  }

  is_moving_ = true;
}

bool Controller::setDest(urs_wearable::SetDest::Request& req, urs_wearable::SetDest::Response& res)
{
  setDest(req.dest, req.set_orientation);
  return true;
}

void Controller::setAltitude(double z)
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  this->dest_.position.z = z;
}

bool Controller::setAltitude(urs_wearable::SetAltitude::Request& req, urs_wearable::SetAltitude::Response& res)
{
  setAltitude(req.z);
  return true;
}

void Controller::navigate(const urs_wearable::PoseEuler& dest, bool set_orientation)
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
    urs_wearable::PoseEuler dest = this->getDest();
    dest.orientation.z = yaw_to_dest;
    this->setDest(dest, true);

    do
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (getYawDiff(this->getPose().orientation.z, yaw_to_dest) > max_orientation_error_ * M_PI / 180.0);
  }

  this->setDest(dest, set_orientation);
}

void Controller::stop()
{
  urs_wearable::PoseEuler pose = getPose();
  setDest(pose, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void Controller::setSpeed(const PID& pid)
{
  std::lock_guard<std::mutex> lock(pid_mutex_);
  pid_ = pid;
}

void Controller::readSonarHeight(const sensor_msgs::RangeConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(sonar_height_mutex_);
  sonar_height_range_ = msg->range;
}

void Controller::readSonarUpward(const sensor_msgs::RangeConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(sonar_upward_mutex_);
  sonar_upward_range_ = msg->range;
}

double Controller::getDistance(const urs_wearable::PoseEuler& from, const urs_wearable::PoseEuler& to)
{
  return std::sqrt(
      (to.position.x - from.position.x) * (to.position.x - from.position.x)
      + (to.position.y - from.position.y) * (to.position.y - from.position.y)
      + (to.position.z - from.position.z) * (to.position.z - from.position.z));
}

double Controller::getYawDiff(double yaw1, double yaw2)   // yaw1 and yaw2 are in radian
{
  double diff = std::fabs(yaw1 - yaw2);
  return (diff > M_PI)? M_PI + M_PI - diff: diff;
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
