#include <thread>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include "urs_wearable/common.h"
#include "urs_wearable/controller.h"

Controller::Controller()
{
  geometry_msgs::PoseStamped::ConstPtr pose_stamped = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("ground_truth_to_tf/pose");
  dest_ = pose_stamped->pose;

  ros::NodeHandle nh;
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  pose_sub_ = nh.subscribe("ground_truth_to_tf/pose", 1, &Controller::pidControl, this);
  depth_image_sub_ = nh.subscribe("camera/depth/image_raw", 1, &Controller::readDepthImage, this);
  sonar_height_sub_ = nh.subscribe<sensor_msgs::Range>("sonar_height", 1, &Controller::readSonarHeight, this);
  sonar_upward_sub_ = nh.subscribe<sensor_msgs::Range>("sonar_upward", 1, &Controller::readSonarUpward, this);

  get_dest_service_ = nh.advertiseService("get_dest", &Controller::getDestService, this);
  set_altitude_service_ = nh.advertiseService("set_altitude", &Controller::setAltitudeService, this);
  set_orientation_service_ = nh.advertiseService("set_orientation", &Controller::setOrientationService, this);
  set_position_service_ = nh.advertiseService("set_position", &Controller::setPositionService, this);
  set_position_bare_service_ = nh.advertiseService("set_position_bare", &Controller::setPositionBareService, this);
  stop_service_ = nh.advertiseService("stop", &Controller::stopService, this);

/* Uncomment to test PID values
  set_p_service_ = nh.advertiseService("set_p", &Controller::setP, this);
  set_i_service_ = nh.advertiseService("set_i", &Controller::setI, this);
  set_d_service_ = nh.advertiseService("set_d", &Controller::setD, this);
  set_pq_service_ = nh.advertiseService("set_pq", &Controller::setPQ, this);
  set_dq_service_ = nh.advertiseService("set_dq", &Controller::setDQ, this);
*/
}

/* Uncomment to test PID values
bool Controller::setP(urs_wearable::SetDouble::Request& req, urs_wearable::SetDouble::Response& res)
{
  {
    std::lock_guard<std::mutex> lock(p_mutex_);
    p_ = req.value;
  }
  ros_warn("[" + std::to_string(p_) + ", " + std::to_string(i_) + ", " + std::to_string(d_) + ", " + std::to_string(pq_) + ", " + std::to_string(dq_) + "]");
  return true;
}
bool Controller::setI(urs_wearable::SetDouble::Request& req, urs_wearable::SetDouble::Response& res)
{
  {
    std::lock_guard<std::mutex> lock(i_mutex_);
    i_ = req.value;
    position_integral_.x = 0.0;
    position_integral_.y = 0.0;
    position_integral_.z = 0.0;
  }
  ros_warn("[" + std::to_string(p_) + ", " + std::to_string(i_) + ", " + std::to_string(d_) + ", " + std::to_string(pq_) + ", " + std::to_string(dq_) + "]");
  return true;
}
bool Controller::setD(urs_wearable::SetDouble::Request& req, urs_wearable::SetDouble::Response& res)
{
  {
    std::lock_guard<std::mutex> lock(d_mutex_);
    d_ = req.value;
  }
  ros_warn("[" + std::to_string(p_) + ", " + std::to_string(i_) + ", " + std::to_string(d_) + ", " + std::to_string(pq_) + ", " + std::to_string(dq_) + "]");
  return true;
}
bool Controller::setPQ(urs_wearable::SetDouble::Request& req, urs_wearable::SetDouble::Response& res)
{
  {
    std::lock_guard<std::mutex> lock(pq_mutex_);
    pq_ = req.value;
  }
  ros_warn("[" + std::to_string(p_) + ", " + std::to_string(i_) + ", " + std::to_string(d_) + ", " + std::to_string(pq_) + ", " + std::to_string(dq_) + "]");
  return true;
}
bool Controller::setDQ(urs_wearable::SetDouble::Request& req, urs_wearable::SetDouble::Response& res)
{
  {
    std::lock_guard<std::mutex> lock(dq_mutex_);
    dq_ = req.value;
  }
  ros_warn("[" + std::to_string(p_) + ", " + std::to_string(i_) + ", " + std::to_string(d_) + ", " + std::to_string(pq_) + ", " + std::to_string(dq_) + "]");
  return true;
}
*/

void Controller::pidControl(const geometry_msgs::PoseStampedConstPtr& pose_stamped)
{
/* Uncomment to test PID values
  double P, I, D, PQ, DQ;
  {
    std::lock_guard<std::mutex> lock(p_mutex_);
    P = p_;
  }
  {
    std::lock_guard<std::mutex> lock(i_mutex_);
    I = i_;
  }
  {
    std::lock_guard<std::mutex> lock(d_mutex_);
    D = d_;
  }
  {
    std::lock_guard<std::mutex> lock(pq_mutex_);
    PQ = pq_;
  }
  {
    std::lock_guard<std::mutex> lock(dq_mutex_);
    DQ = dq_;
  }
*/

  geometry_msgs::Point position_error;
//  geometry_msgs::Quaternion q_error;
  double yaw = quaternionToYaw(pose_stamped->pose.orientation);
  double yaw_error;

  {
    std::lock_guard<std::mutex> lock(dest_mutex_);

    if (pointDistance3D(pose_stamped->pose.position, dest_.position) < MAX_POSITION_ERROR)
    {
      is_moving_ = false;
    }

    position_error.x = dest_.position.x - pose_stamped->pose.position.x;
    position_error.y = dest_.position.y - pose_stamped->pose.position.y;
    position_error.z = dest_.position.z - pose_stamped->pose.position.z;

//    q_error.w = + dest_.orientation.w * pose_stamped->pose.orientation.w
//                    + dest_.orientation.x * pose_stamped->pose.orientation.x
//                    + dest_.orientation.y * pose_stamped->pose.orientation.y
//                    + dest_.orientation.z * pose_stamped->pose.orientation.z;
//    q_error.x = - dest_.orientation.w * pose_stamped->pose.orientation.x
//                    + dest_.orientation.x * pose_stamped->pose.orientation.w
//                    - dest_.orientation.y * pose_stamped->pose.orientation.z
//                    + dest_.orientation.z * pose_stamped->pose.orientation.y;
//    q_error.y = - dest_.orientation.w * pose_stamped->pose.orientation.y
//                    + dest_.orientation.x * pose_stamped->pose.orientation.z
//                    + dest_.orientation.y * pose_stamped->pose.orientation.w
//                    - dest_.orientation.z * pose_stamped->pose.orientation.x;
//    q_error.z = - dest_.orientation.w * pose_stamped->pose.orientation.z
//                    - dest_.orientation.x * pose_stamped->pose.orientation.y
//                    + dest_.orientation.y * pose_stamped->pose.orientation.x
//                    + dest_.orientation.z * pose_stamped->pose.orientation.w;
    yaw_error = quaternionToYaw(dest_.orientation) - yaw;
  }

//  if (q_error.w < 0.0)
//  {
//    q_error.x *= -1;
//    q_error.y *= -1;
//    q_error.z *= -1;
//  }
  if (yaw_error < -M_PI)
  {
    yaw_error += M_PI + M_PI;
  }
  else if (yaw_error > M_PI)
  {
    yaw_error -= M_PI + M_PI;
  }

  geometry_msgs::Point proportional;
  geometry_msgs::Point derivation;
  proportional.x = P * position_error.x;
  proportional.y = P * position_error.y;
  proportional.z = P * position_error.z;
  position_integral_.x += I * position_error.x;
  position_integral_.y += I * position_error.y;
  position_integral_.z += I * position_error.z;
  derivation.x = D * (position_error.x - position_prev_error_.x);
  derivation.y = D * (position_error.y - position_prev_error_.y);
  derivation.z = D * (position_error.z - position_prev_error_.z);

  double x = proportional.x + position_integral_.x + derivation.x;
  double y = proportional.y + position_integral_.y + derivation.y;
  double z = proportional.z + position_integral_.z + derivation.z;
  double rx = x * std::cos(-yaw) - y * std::sin(-yaw);
  double ry = x * std::sin(-yaw) + y * std::cos(-yaw);

  geometry_msgs::Twist cmd;
  cmd.linear.x = rx;
  cmd.linear.y = ry;
  cmd.linear.z = z;

//  sensor_msgs::Imu::ConstPtr imu = ros::topic::waitForMessage<sensor_msgs::Imu>("raw_imu");
//  cmd.angular.x = - PQ * q_error.x - DQ * imu->angular_velocity.x;
//  cmd.angular.y = - PQ * q_error.y - DQ * imu->angular_velocity.y;
//  cmd.angular.z = PQ * q_error.z + DQ * imu->angular_velocity.z;
  cmd.angular.z = PQ * yaw_error + DQ * (yaw_error - yaw_prev_error_);

  cmd_pub_.publish(cmd);

  position_prev_error_ = position_error;
  yaw_prev_error_ = yaw_error;
}

void Controller::readDepthImage(const sensor_msgs::Image::ConstPtr& msg)
{
  /* https://answers.ros.org/question/246066/how-can-i-get-object-distance-using-cameradepthimage_raw
   * depth_array[0], depth_array[1], ..., depth_aray[image.height * image.width - 1] */
  if (is_moving_ && msg->encoding.compare("32FC1") == 0)
  {
    const float* depth_image_array = reinterpret_cast<const float*>(&(msg->data[0]));
    geometry_msgs::PoseStamped::ConstPtr pose_stamped = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("ground_truth_to_tf/pose");

    if (pose_stamped->pose.position.z > 1.0)
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
            if (pointDistance2D(pose_stamped->pose.position, getDest().position) > depth)
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

  const geometry_msgs::Pose curr_dest = getDest();

  stop();
  ros::Duration(1.0).sleep();

  geometry_msgs::PoseStamped::ConstPtr pose_stamped = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("ground_truth_to_tf/pose");
  geometry_msgs::Pose pose = pose_stamped->pose;
  pose.position.z += 2.0;
  setPosition(pose.position);
  ros::Duration(1.0).sleep();

  setPosition(curr_dest.position);
}

void Controller::stop()
{
  geometry_msgs::PoseStamped::ConstPtr pose_stamped = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("ground_truth_to_tf/pose");
  setPositionBare(pose_stamped->pose.position);
}

void Controller::readSonarHeight(const sensor_msgs::RangeConstPtr& msg)
{
  float sonar_height_range = msg->range;
  // TODO: Check for collision
}

void Controller::readSonarUpward(const sensor_msgs::RangeConstPtr& msg)
{
  float sonar_upward_range = msg->range;
  // TODO: Check for collision
}

geometry_msgs::Pose Controller::getDest()
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  return dest_;
}

void Controller::setAltitude(double height)
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  dest_.position.z = height;
}

void Controller::setOrientation(const double yaw)   // yaw in radian
{
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);

  std::lock_guard<std::mutex> lock(dest_mutex_);
  dest_.orientation.x = q.x();
  dest_.orientation.y = q.y();
  dest_.orientation.z = q.z();
  dest_.orientation.w = q.w();
}

void Controller::setPosition(const geometry_msgs::Point position)
{
  geometry_msgs::PoseStamped::ConstPtr pose_stamped = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("ground_truth_to_tf/pose");
  double vx = position.x - pose_stamped->pose.position.x;
  double vy = position.y - pose_stamped->pose.position.y;
  double yaw_to_dest = std::atan2(vy, vx);

  double planar_dist_to_dest = std::sqrt(vx * vx + vy * vy);
  if (planar_dist_to_dest > MAX_POSITION_ERROR)
  {
    setOrientation(yaw_to_dest);
    do
    {
      pose_stamped = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("ground_truth_to_tf/pose");
    } while (yawDiff(quaternionToYaw(pose_stamped->pose.orientation), yaw_to_dest) > MAX_ORIENTATION_ERROR * M_PI / 180.0);
  }

  setPositionBare(position);
  is_moving_ = true;
}

void Controller::setPositionBare(const geometry_msgs::Point& position)
{
  std::lock_guard<std::mutex> lock(dest_mutex_);
  dest_.position.x = position.x;
  dest_.position.y = position.y;
  dest_.position.z = position.z;
}

bool Controller::getDestService(urs_wearable::GetDest::Request& req, urs_wearable::GetDest::Response& res)
{
  res.dest = getDest();
  return true;
}

bool Controller::setAltitudeService(urs_wearable::SetAltitude::Request& req, urs_wearable::SetAltitude::Response& res)
{
  setAltitude(req.height);
  return true;
}

bool Controller::setOrientationService(urs_wearable::SetOrientation::Request& req, urs_wearable::SetOrientation::Response& res)
{
  setOrientation(req.yaw);
  return true;
}

bool Controller::setPositionService(urs_wearable::SetPosition::Request& req, urs_wearable::SetPosition::Response& res)
{
  std::thread t(&Controller::setPosition, this, req.position);
  t.detach();
  return true;
}

bool Controller::setPositionBareService(urs_wearable::SetPosition::Request& req, urs_wearable::SetPosition::Response& res)
{
  setPositionBare(req.position);
  return true;
}

bool Controller::stopService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  stop();
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  // Make sure internal controller of hector_quadrotor is running
  if (!ros::service::waitForService("controller/attitude/yawrate/set_parameters", -1) ||
      !ros::service::waitForService("controller/position/z/set_parameters", -1) ||
      !ros::service::waitForService("controller/velocity/z/set_parameters", -1))
  {
    ros_error("The drone has not been spawned");
  }

  Controller controller;

  ros::spin();

  return EXIT_SUCCESS;
}
