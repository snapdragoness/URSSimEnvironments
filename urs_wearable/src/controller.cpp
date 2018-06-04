#define ENABLE_MOTORS_SERVICE "/enable_motors"

#include "tf/transform_datatypes.h"

#include "urs_wearable/controller.h"

Controller::~Controller()
{
  poseSub.shutdown();

  cmdPub.shutdown();
  posePubThread.interrupt();
}

Controller::Controller(const std::string& ns)
{
  this->ns = ns;

  // set PID values
  pid.p = 0.5;  // default: 0.5
  pid.i = 0.0;
  pid.d = 0.0;

  error.position.x = error.position.y = error.position.z = error.yaw = 0.0;
  integral.position.x = integral.position.y = integral.position.z = integral.yaw = 0.0;
}

void Controller::start(double height)
{
  // create a service client
  ros::ServiceClient serviceClient = nh->serviceClient<hector_uav_msgs::EnableMotors>(ns + ENABLE_MOTORS_SERVICE);
  hector_uav_msgs::EnableMotors enable_motors_srv;
  enable_motors_srv.request.enable = true;

  if (serviceClient.call(enable_motors_srv))
  {
    ROS_INFO("%s - %s", nh->resolveName(ns + ENABLE_MOTORS_SERVICE).c_str(),
             enable_motors_srv.response.success ? "successful" : "failed");

    // get initial position
    geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(ns + "/ground_truth_to_tf/pose");

    // start at 1 meter above its starting point while preserve the orientation
    dest.position.x = msg->pose.position.x;
    dest.position.y = msg->pose.position.y;
    dest.position.z = msg->pose.position.z + height;
    dest.yaw = Controller::quaternionToYaw(boost::make_shared<geometry_msgs::Quaternion>(msg->pose.orientation));

    poseSub = nh->subscribe(ns + "/ground_truth_to_tf/pose", 1, &Controller::controller, this);
    cmdPub = nh->advertise<geometry_msgs::Twist>(ns + "/cmd_vel", 1, false);

    posePubThread = boost::thread(&Controller::_posePub, this, 10);

    controllerSetDest = nh->advertiseService(ns + "/set_dest", &Controller::setDest, this);
  }
  else
  {
    ROS_ERROR("Failed to call %s", nh->resolveName(ns + ENABLE_MOTORS_SERVICE).c_str());
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
    yaw += 2 * M_PI;
  }

  mut_pose.lock();
  pose.position.x = msg->pose.position.x;
  pose.position.y = msg->pose.position.y;
  pose.position.z = msg->pose.position.z;
  pose.yaw = yaw;
//  pose.pitch = pitch;
  mut_pose.unlock();

  prevError.position.x = error.position.x;
  prevError.position.y = error.position.y;
  prevError.position.z = error.position.z;
  prevError.yaw = error.yaw;

  mut_dest.lock();
  error.position.x = dest.position.x - msg->pose.position.x;
  error.position.y = dest.position.y - msg->pose.position.y;
  error.position.z = dest.position.z - msg->pose.position.z;
  error.yaw = dest.yaw - yaw;
  mut_dest.unlock();

  if (error.yaw < -M_PI) {
    error.yaw += M_PI + M_PI;
  } else if (error.yaw > M_PI) {
    error.yaw -= M_PI + M_PI;
  }

  proportional.position.x = pid.p * error.position.x;
  proportional.position.y = pid.p * error.position.y;
  proportional.position.z = pid.p * error.position.z;
  integral.position.x += pid.i * error.position.x;
  integral.position.y += pid.i * error.position.y;
  integral.position.z += pid.i * error.position.z;
  derivation.position.x = pid.d * (error.position.x - prevError.position.x);
  derivation.position.y = pid.d * (error.position.y - prevError.position.y);
  derivation.position.z = pid.d * (error.position.z - prevError.position.z);

  double x, y, z;
  x = proportional.position.x + integral.position.x + derivation.position.x;
  y = proportional.position.y + integral.position.y + derivation.position.y;
  z = proportional.position.z + integral.position.z + derivation.position.z;

  double rx, ry;
  rx = x * std::cos(-1 * yaw) - y * std::sin(-1 * yaw);
  ry = x * std::sin(-1 * yaw) + y * std::cos(-1 * yaw);

  cmd.linear.x = rx;
  cmd.linear.y = ry;
  cmd.linear.z = z;
  cmd.angular.z = 2.0 * error.yaw;

  cmdPub.publish(cmd);
}

void Controller::_posePub(ros::Rate rate)
{
  ros::Publisher posePub = nh->advertise<urs_wearable::PoseEuler>(ns + "/urs_wearable/pose", 10, false);

  while (ros::ok())
  {
    try
    {
      urs_wearable::PoseEuler pose;

      mut_pose.lock();
      pose.position.x = this->pose.position.x;
      pose.position.y = this->pose.position.y;
      pose.position.z = this->pose.position.z;
      pose.yaw = this->pose.yaw;
      mut_pose.unlock();

      posePub.publish(pose);

      ros::spinOnce();
      rate.sleep();
    }
    catch (boost::thread_interrupted&)
    {
      break;
    }
  }

  /* clean up the publisher */
  posePub.shutdown();
}

urs_wearable::PoseEuler Controller::getPose()
{
  mut_pose.lock();
  urs_wearable::PoseEuler pose = this->pose;
  mut_pose.unlock();
  return pose;
}

urs_wearable::PoseEuler Controller::getDest()
{
  mut_dest.lock();
  urs_wearable::PoseEuler dest = this->dest;
  mut_dest.unlock();
  return dest;
}

void Controller::setDest(const urs_wearable::PoseEuler& dest, bool set_orientation)
{
  mut_dest.lock();
  this->dest.position.x = dest.position.x;
  this->dest.position.y = dest.position.y;
  this->dest.position.z = dest.position.z;
  if (set_orientation)
  {
    this->dest.yaw = dest.yaw;
  }
  mut_dest.unlock();
}

bool Controller::setDest(urs_wearable::ControllerSetDest::Request& req, urs_wearable::ControllerSetDest::Response& res)
{
  mut_dest.lock();
  this->dest.position.x = req.dest.pose.position.x;
  this->dest.position.y = req.dest.pose.position.y;
  this->dest.position.z = req.dest.pose.position.z;
  if (req.dest.set_orientation)
  {
    this->dest.yaw = req.dest.pose.yaw;
  }
  mut_dest.unlock();
  return true;
}

void Controller::setNamespace(const std::string& ns)
{
  this->ns = ns;
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
