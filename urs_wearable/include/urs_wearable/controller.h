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
#include <urs_wearable/GetDest.h>
#include <urs_wearable/GetPose.h>
#include <urs_wearable/PoseEuler.h>
#include <urs_wearable/SetDest.h>
#include <urs_wearable/SetAltitude.h>

typedef struct PID
{
  double p, i, d;
} PID;

class Controller {
  // read about adjusting PID coefficients at https://oscarliang.com/quadcopter-pid-explained-tuning
  // {0.5, 0.0, 0.0} for perfect environment
  // {0.5, 0.0002, 0.00005} for imperfect environment
  PID pid_;
  const PID pid_default_ = {0.2, 0.0, 0.0};
  const PID pid_slow_ = {0.1, 0.0, 0.0};

  urs_wearable::PoseEuler error_;
  urs_wearable::PoseEuler prev_error_;
  urs_wearable::PoseEuler proportional_;
  urs_wearable::PoseEuler integral_;
  urs_wearable::PoseEuler derivation_;

  urs_wearable::PoseEuler pose_;   // position of the UAV from ground truth
  urs_wearable::PoseEuler dest_;   // desired destination of the UAV

  ros::Publisher cmd_pub_;
  ros::Subscriber pose_sub_;      // a subscriber of the UAV's ground truth
  ros::ServiceServer get_dest_service_;
  ros::ServiceServer get_pose_service_;
  ros::ServiceServer set_dest_service_;
  ros::ServiceServer set_altitude_service_;

  std::mutex dest_mutex_;
  std::mutex pose_mutex_;

  void pidControl(const geometry_msgs::PoseStampedConstPtr& msg);
  void poseEulerPublish();

  ros::Subscriber depth_image_sub_;
  void readDepthImage(const sensor_msgs::Image::ConstPtr&);
  void avoidObstacle();
  std::atomic<bool> is_moving_{false};

  void readSonarHeight(const sensor_msgs::RangeConstPtr&);
  void readSonarUpward(const sensor_msgs::RangeConstPtr&);
  ros::Subscriber sonar_height_sub_;
  ros::Subscriber sonar_upward_sub_;
  float sonar_height_range_ = 0.0;
  float sonar_upward_range_ = 0.0;

  void stop();
  void setSpeed(const PID&);
  std::mutex pid_mutex_;

  urs_wearable::PoseEuler getDest();
  urs_wearable::PoseEuler getPose();
  void setDest(const urs_wearable::PoseEuler&, bool);
  void setAltitude(const double);

public:
  Controller();
  ~Controller();

  bool getDestService(urs_wearable::GetDest::Request&, urs_wearable::GetDest::Response&);
  bool getPoseService(urs_wearable::GetPose::Request&, urs_wearable::GetPose::Response&);
  bool setDest(urs_wearable::SetDest::Request&, urs_wearable::SetDest::Response&);
  bool setAltitude(urs_wearable::SetAltitude::Request&, urs_wearable::SetAltitude::Response&);

  void navigate(const urs_wearable::PoseEuler&, bool);

  const double max_position_error_ = 0.2;      // maximum position error in meter (only non-negative values)
  const double max_orientation_error_ = 1.0;   // maximum orientation error in degree (only non-negative values)
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_ */
