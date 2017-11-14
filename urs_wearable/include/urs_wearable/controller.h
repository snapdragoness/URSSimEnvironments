#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_

#include "urs_wearable/pose.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>

#include <boost/thread.hpp>

typedef struct PID
{
  double p, i, d;
} PID;

class Controller {
  ros::NodeHandlePtr nh;
  std::string ns;         // namespace of the UAV

  // read about adjusting PID coefficients at https://oscarliang.com/quadcopter-pid-explained-tuning/
  // {0.5, 0.0, 0.0} for perfect environment
  // {0.5, 0.0002, 0.00005} for imperfect environment
  PID pid;
  Pose error;
  Pose prevError;
  Pose proportional;
  Pose integral;
  Pose derivation;

  Pose pose;    // position of the UAV from ground truth
  Pose dest;    // desired destination of the UAV

  ros::Subscriber poseSub;    // subscriber of the UAV's ground truth
  geometry_msgs::Twist cmd;   // a twist message to be sent to /cmd_vel

  boost::thread commanderThread;

  boost::mutex mut_dest;
  boost::mutex mut_pose;
  boost::mutex mut_cmd;

  // methods with an underscore in front are supposed to be created as threads
  void _controller(const geometry_msgs::PoseStampedConstPtr& msg);
  void _commander(ros::Rate rate);

public:
  Controller(const std::string& ns);
  Controller() : Controller("") {}
  ~Controller();

  // important methods
  void init() {
    nh = boost::make_shared<ros::NodeHandle>();
  }
  void start(double height = 1.0);

  // auxiliary methods
  Pose getPose();
  Pose getDest();
  void setDest(const Pose& dest);
  void setDest(double x, double y, double z);
  void setNamespace(const std::string& ns);

  // static methods
  static double quaternionToYaw(const geometry_msgs::QuaternionConstPtr& q);
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_ */
