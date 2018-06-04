#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_

#include "urs_wearable/PoseEuler.h"

#include "urs_wearable/ControllerSetDest.h"
#include "urs_wearable/GetPose.h"
#include "urs_wearable/SetDest.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <sensor_msgs/Image.h>

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
  urs_wearable::PoseEuler error;
  urs_wearable::PoseEuler prevError;
  urs_wearable::PoseEuler proportional;
  urs_wearable::PoseEuler integral;
  urs_wearable::PoseEuler derivation;

  urs_wearable::PoseEuler pose;   // position of the UAV from ground truth
  urs_wearable::PoseEuler dest;   // desired destination of the UAV

  ros::Subscriber poseSub;        // subscriber of the UAV's ground truth
  ros::Subscriber depthImageSub;  // subscriber of the UAV's depth image
  geometry_msgs::Twist cmd;       // a twist message to be sent to /cmd_vel

  ros::ServiceServer controllerSetDest;

  boost::thread commanderThread;
  boost::thread posePubThread;

  boost::mutex mut_dest;
  boost::mutex mut_pose;

  void controller(const geometry_msgs::PoseStampedConstPtr& msg);

  // methods with an underscore in front are supposed to be created as threads
  void _posePub(ros::Rate rate);

  ros::Publisher cmdPub;

public:
  Controller(const std::string& ns);
  Controller() : Controller("") {}
  ~Controller();

  // important methods
  void init() {
    nh = boost::make_shared<ros::NodeHandle>();
  }
  void start(double height = 5.0);

  // auxiliary methods
  urs_wearable::PoseEuler getPose();
  urs_wearable::PoseEuler getDest();
  void setDest(const urs_wearable::PoseEuler& dest, bool rotate);
  bool setDest(urs_wearable::ControllerSetDest::Request& req, urs_wearable::ControllerSetDest::Response& res);
  void setNamespace(const std::string& ns);

  // static methods
  static double quaternionToYaw(const geometry_msgs::QuaternionConstPtr&);
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_ */
