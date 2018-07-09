#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_

#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <sensor_msgs/Image.h>

#include "urs_wearable/PoseEuler.h"
#include "urs_wearable/SetDest.h"

typedef struct PID
{
  double p, i, d;
} PID;

class Controller {
  std::string ns_;
  bool has_set_ns_;

  // read about adjusting PID coefficients at https://oscarliang.com/quadcopter-pid-explained-tuning/
  // {0.5, 0.0, 0.0} for perfect environment
  // {0.5, 0.0002, 0.00005} for imperfect environment
  PID pid_;
  urs_wearable::PoseEuler error_;
  urs_wearable::PoseEuler prev_error_;
  urs_wearable::PoseEuler proportional_;
  urs_wearable::PoseEuler integral_;
  urs_wearable::PoseEuler derivation_;

  urs_wearable::PoseEuler pose;   // position of the UAV from ground truth
  urs_wearable::PoseEuler dest;   // desired destination of the UAV

  ros::Publisher cmd_pub_;
  ros::Subscriber pose_sub_;      // a subscriber of the UAV's ground truth
  ros::ServiceServer set_dest_service_;

  geometry_msgs::Twist cmd;       // a twist message to be sent to /cmd_vel

  std::mutex dest_mutex_;
  std::mutex pose_mutex_;

  void controller(const geometry_msgs::PoseStampedConstPtr& msg);
  void poseEulerPublish(ros::NodeHandle&, ros::Rate rate);

public:
  Controller();
  ~Controller();

  bool setNamespace(ros::NodeHandle&, const std::string&);

  urs_wearable::PoseEuler getPose();
  urs_wearable::PoseEuler getDest();
  void setDest(const urs_wearable::PoseEuler&, bool);
  bool setDest(urs_wearable::SetDest::Request&, urs_wearable::SetDest::Response&);

  static double quaternionToYaw(const geometry_msgs::QuaternionConstPtr&);
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_ */
