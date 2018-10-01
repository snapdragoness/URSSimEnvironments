#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_

#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>

#include "urs_wearable/PoseEuler.h"
#include "urs_wearable/SetDest.h"
#include "urs_wearable/SetAltitude.h"

typedef struct PID
{
  double p, i, d;
} PID;

class Controller {
  std::string ns_;
  bool has_set_ns_;

  PID pid_default_;
  PID pid_slow_;

  // read about adjusting PID coefficients at https://oscarliang.com/quadcopter-pid-explained-tuning/
  // {0.5, 0.0, 0.0} for perfect environment
  // {0.5, 0.0002, 0.00005} for imperfect environment
  PID pid_;
  urs_wearable::PoseEuler error_;
  urs_wearable::PoseEuler prev_error_;
  urs_wearable::PoseEuler proportional_;
  urs_wearable::PoseEuler integral_;
  urs_wearable::PoseEuler derivation_;

  urs_wearable::PoseEuler pose_;   // position of the UAV from ground truth
  urs_wearable::PoseEuler dest_;   // desired destination of the UAV

  ros::Publisher cmd_pub_;
  ros::Subscriber pose_sub_;      // a subscriber of the UAV's ground truth
  ros::ServiceServer set_dest_service_;
  ros::ServiceServer set_altitude_service_;

  geometry_msgs::Twist cmd;       // a twist message to be sent to /cmd_vel

  std::mutex dest_mutex_;
  std::mutex pose_mutex_;

  void controller(const geometry_msgs::PoseStampedConstPtr& msg);
  void poseEulerPublish(ros::NodeHandle&, ros::Rate rate);

  ros::Subscriber depth_image_sub_;
  void readDepthImage(const sensor_msgs::Image::ConstPtr&);
  void avoidObstacle();
  std::atomic<bool> is_moving_{false};

  std::mutex depth_image_array_mutex_;
  const float* depth_image_array_;
  const int n_samples = 1000;

  void readSonarHeight(const sensor_msgs::RangeConstPtr&);
  void readSonarUpward(const sensor_msgs::RangeConstPtr&);
  ros::Subscriber sonar_height_sub_;
  ros::Subscriber sonar_upward_sub_;
  float sonar_height_range_ = 0.0;
  float sonar_upward_range_ = 0.0;
  std::mutex sonar_height_mutex_;
  std::mutex sonar_upward_mutex_;

  void stop();
  void setSpeed(const PID&);
  std::mutex pid_mutex_;

public:
  Controller();
  ~Controller();

  bool setNamespace(ros::NodeHandle&, const std::string&);

  urs_wearable::PoseEuler getPose();
  urs_wearable::PoseEuler getDest();
  void setDest(const urs_wearable::PoseEuler&, bool);
  bool setDest(urs_wearable::SetDest::Request&, urs_wearable::SetDest::Response&);
  void setAltitude(const double);
  bool setAltitude(urs_wearable::SetAltitude::Request&, urs_wearable::SetAltitude::Response&);

  void navigate(const urs_wearable::PoseEuler&, bool);
  void printImage();

  static double quaternionToYaw(const geometry_msgs::QuaternionConstPtr&);
  static double getDistance(const urs_wearable::PoseEuler&, const urs_wearable::PoseEuler&);
  static double getYawDiff(double, double);

  const double max_position_error_ = 0.2;      // maximum position error in meter (only non-negative values)
  const double max_orientation_error_ = 1.0;   // maximum orientation error in degree (only non-negative values)
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_ */
