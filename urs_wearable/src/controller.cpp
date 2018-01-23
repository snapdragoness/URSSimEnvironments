#define ENABLE_MOTORS_SERVICE "/enable_motors"

#include "urs_wearable/controller.h"

Controller::~Controller()
{
  poseSub.shutdown();
  destSub.shutdown();

  commanderThread.interrupt();
  posePubThread.interrupt();
}

Controller::Controller(const std::string& ns)
{
  this->ns = ns;

  // set PID values
  pid.p = 0.5;
  pid.i = 0.0;
  pid.d = 0.0;

  error.x = error.y = error.z = error.yaw = 0.0;
  integral.x = integral.y = integral.z = integral.yaw = 0.0;
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
    dest.x = msg->pose.position.x;
    dest.y = msg->pose.position.y;
    dest.z = msg->pose.position.z + height;
    dest.yaw = Controller::quaternionToYaw(boost::make_shared<geometry_msgs::Quaternion>(msg->pose.orientation));

    poseSub = nh->subscribe(ns + "/ground_truth_to_tf/pose", 10, &Controller::_controller, this);
    destSub = nh->subscribe(ns + "/urs_wearable/dest", 10, &Controller::setDest, this);

    commanderThread = boost::thread(&Controller::_commander, this, 10);
    posePubThread = boost::thread(&Controller::_posePub, this, 10);
  }
  else
  {
    ROS_ERROR("Failed to call %s", nh->resolveName(ns + ENABLE_MOTORS_SERVICE).c_str());
  }
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

void Controller::_controller(const geometry_msgs::PoseStampedConstPtr& msg)
{
  double yaw = quaternionToYaw(boost::make_shared<geometry_msgs::Quaternion>(msg->pose.orientation));

  mut_pose.lock();
  pose.x = msg->pose.position.x;
  pose.y = msg->pose.position.y;
  pose.z = msg->pose.position.z;
  pose.yaw = yaw;
  mut_pose.unlock();

  prevError.x = error.x;
  prevError.y = error.y;
  prevError.z = error.z;
  prevError.yaw = error.yaw;

  mut_dest.lock();
  error.x = dest.x - msg->pose.position.x;
  error.y = dest.y - msg->pose.position.y;
  error.z = dest.z - msg->pose.position.z;
  error.yaw = dest.yaw - yaw;
  mut_dest.unlock();

  if (error.yaw < -M_PI) {
    error.yaw += M_PI + M_PI;
  } else if (error.yaw > M_PI) {
    error.yaw -= M_PI + M_PI;
  }

  proportional.x = pid.p * error.x;
  proportional.y = pid.p * error.y;
  proportional.z = pid.p * error.z;
  integral.x += pid.i * error.x;
  integral.y += pid.i * error.y;
  integral.z += pid.i * error.z;
  derivation.x = pid.d * (error.x - prevError.x);
  derivation.y = pid.d * (error.y - prevError.y);
  derivation.z = pid.d * (error.z - prevError.z);

  double x, y, z;
  x = proportional.x + integral.x + derivation.x;
  y = proportional.y + integral.y + derivation.y;
  z = proportional.z + integral.z + derivation.z;

  double rx, ry;
  rx = x * std::cos(-1 * yaw) - y * std::sin(-1 * yaw);
  ry = x * std::sin(-1 * yaw) + y * std::cos(-1 * yaw);

  mut_cmd.lock();
  cmd.linear.x = rx;
  cmd.linear.y = ry;
  cmd.linear.z = z;
  cmd.angular.z = 2.0 * error.yaw;
  mut_cmd.unlock();
}

void Controller::_commander(ros::Rate rate)
{
  ros::Publisher cmdPub = nh->advertise<geometry_msgs::Twist>(ns + "/cmd_vel", 10, false);
  while (ros::ok())
  {
    try
    {
      mut_cmd.lock();
      cmdPub.publish(cmd);
      mut_cmd.unlock();
      ros::spinOnce();
      rate.sleep();
    }
    catch (boost::thread_interrupted&)
    {
      break;
    }
  }

  /* clean up the publisher */
  cmdPub.shutdown();
}

void Controller::_posePub(ros::Rate rate)
{
  ros::Publisher posePub = nh->advertise<urs_wearable::Pose>(ns + "/urs_wearable/pose", 10, false);

  while (ros::ok())
  {
    try
    {
      urs_wearable::Pose pose;

      mut_pose.lock();
      pose.x = this->pose.x;
      pose.y = this->pose.y;
      pose.z = this->pose.z;
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

Pose Controller::getPose()
{
  mut_pose.lock();
  Pose pose = this->pose;
  mut_pose.unlock();
  return pose;
}

Pose Controller::getDest()
{
  mut_dest.lock();
  Pose dest = this->dest;
  mut_dest.unlock();
  return dest;
}

void Controller::setDest(const Pose& dest, bool rotate)
{
  mut_dest.lock();
  this->dest.x = dest.x;
  this->dest.y = dest.y;
  this->dest.z = dest.z;
  if (rotate)
  {
    this->dest.yaw = dest.yaw;
  }
  mut_dest.unlock();
}

void Controller::setDest(const urs_wearable::PoseConstPtr& dest)
{
  mut_dest.lock();
  this->dest.x = dest->x;
  this->dest.y = dest->y;
  this->dest.z = dest->z;
  if (dest->rotate)
  {
    this->dest.yaw = dest->yaw;
  }
  mut_dest.unlock();
}

void Controller::setNamespace(const std::string& ns)
{
  this->ns = ns;
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
