#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>

#include <hector_uav_msgs/EnableMotors.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <boost/thread.hpp>

#define ENABLE_MOTORS_SERVICE "/enable_motors"

typedef struct PID
{
  double p;
  double i;
  double d;
} PID;

typedef struct Euler
{
  double roll;
  double pitch;
  double yaw;
} Euler;

bool destReached = false;

PID pidConst = {0.5, 0.0002, 0.00005};
const double tolerance = 0.5;

// real location of the UAVs
geometry_msgs::Pose uavPose;

// manually-set destinations for nUAV
geometry_msgs::Pose dest;

/* for PID control */
geometry_msgs::Pose error;
geometry_msgs::Pose prevError;
geometry_msgs::Pose proportional;
geometry_msgs::Pose integral;
geometry_msgs::Pose derivation;

boost::mutex mut;

geometry_msgs::Twist cmd;
ros::Publisher pub_topic;

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

// PID Control
void uavController(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  Euler euler = quaternionToEuler(boost::make_shared<geometry_msgs::Quaternion>(msg->pose.orientation));

  uavPose.position.x = msg->pose.position.x;
  uavPose.position.y = msg->pose.position.y;
  uavPose.position.z = msg->pose.position.z;
  uavPose.orientation.z = msg->pose.orientation.z;

  prevError.position.x = error.position.x;
  prevError.position.y = error.position.y;
  prevError.position.z = error.position.z;
  prevError.orientation.z = error.orientation.z;
  mut.lock();
  error.position.x = dest.position.x - uavPose.position.x;
  error.position.y = dest.position.y - uavPose.position.y;
  error.position.z = dest.position.z - uavPose.position.z;
  error.orientation.z = dest.orientation.z - uavPose.orientation.z;
  mut.unlock();

  proportional.position.x = pidConst.p * error.position.x;
  proportional.position.y = pidConst.p * error.position.y;
  proportional.position.z = pidConst.p * error.position.z;
  proportional.orientation.z = pidConst.p * error.orientation.z;
  integral.position.x += pidConst.i * error.position.x;
  integral.position.y += pidConst.i * error.position.y;
  integral.position.z += pidConst.i * error.position.z;
  integral.orientation.z += pidConst.i * error.orientation.z;
  derivation.position.x = pidConst.d * (error.position.x - prevError.position.x);
  derivation.position.y = pidConst.d * (error.position.y - prevError.position.y);
  derivation.position.z = pidConst.d * (error.position.z - prevError.position.z);
  derivation.orientation.z = pidConst.d * (error.orientation.z - prevError.orientation.z);

  double x, y, z, oz;
  x = proportional.position.x + integral.position.x + derivation.position.x;
  y = proportional.position.y + integral.position.y + derivation.position.y;
  z = proportional.position.z + integral.position.z + derivation.position.z;
  oz = proportional.orientation.z + integral.orientation.z + derivation.orientation.z;

  double rx, ry;
  rx = x * cos(-1 * euler.yaw) - y * sin(-1 * euler.yaw);
  ry = x * sin(-1 * euler.yaw) + y * cos(-1 * euler.yaw);

  cmd.linear.x = rx;
  cmd.linear.y = ry;
  cmd.linear.z = z;
  cmd.angular.z = oz;

  pub_topic.publish(cmd);

  if ((std::fabs(error.position.x) < tolerance) && (std::fabs(error.position.y) < tolerance)
      && (std::fabs(error.position.z) < tolerance) && (std::fabs(error.orientation.z) < tolerance))
  {
    destReached = true;
  }
}


class Controller
{
private:
  typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
  typedef ActionServer::GoalHandle GoalHandle;
public:
  Controller(ros::NodeHandle &n) :
      node_(n), action_server_(node_, "multi_dof_joint_trajectory_action", boost::bind(&Controller::goalCB, this, _1),
                               boost::bind(&Controller::cancelCB, this, _1), false), has_active_goal_(false)
  {
    creato = 0;
    empty.linear.x = 0;
    empty.linear.y = 0;
    empty.linear.z = 0;
    empty.angular.z = 0;
    empty.angular.y = 0;
    empty.angular.x = 0;
    action_server_.start();

    ROS_INFO_STREAM("Node ready!");
  }
public:
  ros::NodeHandle node_;
  ActionServer action_server_;

  geometry_msgs::Twist empty;
  geometry_msgs::Transform_<std::allocator<void> > lastPosition;
  pthread_t trajectoryExecutor;
  int creato;

  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

  void cancelCB(GoalHandle gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      if (creato)
      {
        ROS_INFO_STREAM("Stop thread");
        pthread_cancel(trajectoryExecutor);
        creato = 0;
      }
      pub_topic.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }

  void goalCB(GoalHandle gh)
  {
    if (has_active_goal_)
    {
      // Stops the controller.
      if (creato)
      {
        pthread_cancel(trajectoryExecutor);
        creato = 0;
      }
      pub_topic.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
    toExecute = gh.getGoal()->trajectory;

    //controllore solo per il giunto virtuale Base
    if (pthread_create(&trajectoryExecutor, NULL, threadWrapper, this) == 0)
    {
      creato = 1;
      ROS_INFO_STREAM("Thread for trajectory execution created");
    }
    else
    {
      ROS_INFO_STREAM("Thread creation failed!");
    }

  }

  static void* threadWrapper(void* arg)
  {
    Controller * mySelf = (Controller*)arg;
    mySelf->executeTrajectory();
    return NULL;
  }

  void executeTrajectory()
  {
    if (toExecute.joint_names[0] == "Base" && toExecute.points.size() > 0)
    {
      for (int k = 0; k < toExecute.points.size(); k++)
      {
        //ricavo cmd da effettuare
        geometry_msgs::Transform_<std::allocator<void> > punto = toExecute.points[k].transforms[0];

        mut.lock();
        dest.position.x = punto.translation.x;
        dest.position.y = punto.translation.y;
        dest.position.z = punto.translation.z;
        dest.orientation.z = punto.rotation.z;
        mut.unlock();

        destReached = false;

        while (!destReached);

//        bool eseguito = true;
//        if (k != 0)
//        {
//          eseguito = publishTranslationComand(punto, false);
//          if (k == (toExecute.points.size() - 1))
//          {
//            if (!eseguito)
//              publishTranslationComand(punto, true);
//            publishRotationComand(punto, false);
//          }
//        }
//        else
//        {
//          publishRotationComand(punto, true);
//        }
//        pub_topic.publish(empty);
//        //aggiorno start position
//        if (eseguito)
//        {
//          lastPosition.translation = punto.translation;
//          lastPosition.rotation = punto.rotation;
//        }
      }
    }
    active_goal_.setSucceeded();
    has_active_goal_ = false;
    creato = 0;

  }

  bool publishTranslationComand(geometry_msgs::Transform_<std::allocator<void> > punto, bool anyway)
  {
    //creazione comando di traslazione

    cmd.linear.x = punto.translation.x - lastPosition.translation.x;
    cmd.linear.y = punto.translation.y - lastPosition.translation.y;
    cmd.linear.z = punto.translation.z - lastPosition.translation.z;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;

    if (anyway || cmd.linear.x >= 0.5 || cmd.linear.y >= 0.5 || cmd.linear.z >= 0.5)
    {
      printPositionInfo();
      printCmdInfo();
      pub_topic.publish(cmd);
      //tempo d'esecuzione
      ros::Duration(1.0).sleep();
      return true;
    }
    return false;
  }

  void publishRotationComand(geometry_msgs::Transform_<std::allocator<void> > punto, bool start)
  {
    //comando di allineamento, permesse solo rotazioni sull'asse z
    cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.x = cmd.angular.y = 0;
    //start = true --> devo tornare nell'orientazione 0
    //start = false --> devo arrivare al'orientazione punto.rotation.z
    cmd.angular.z = (start ? 0 - punto.rotation.z : punto.rotation.z);

    printCmdInfo();

    double sleep = cmd.angular.z * 3.0; //tempo necessario a tornare nella giusta orientazione
    if (sleep < 0)
      sleep = -sleep;
    pub_topic.publish(cmd);
    ros::Duration(sleep).sleep();
    cmd.angular.z = 0;
  }

  void printPositionInfo()
  {
    ROS_INFO_STREAM(
        "Start Position: ["<<lastPosition.translation.x<< ", "<<lastPosition.translation.y<< ", "<<lastPosition.translation.z<<"] "<< "[ "<<lastPosition.rotation.x<< ", "<<lastPosition.rotation.y<< ", "<<lastPosition.rotation.z<<" ]");
  }

  void printCmdInfo()
  {
    ROS_INFO_STREAM(
        "cmd to execute: "<<"x:"<<cmd.linear.x <<" y: " << cmd.linear.y <<" z: " << cmd.linear.z <<" rX: " << cmd.angular.x <<" rY: " << cmd.angular.y <<" rZ: " << cmd.angular.z);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_controller_node");
  ros::NodeHandle nh; //("~");

  // create a service client
  ros::ServiceClient client;
  hector_uav_msgs::EnableMotors enable_motors_srv;

  client = nh.serviceClient<hector_uav_msgs::EnableMotors>(ENABLE_MOTORS_SERVICE);
  enable_motors_srv.request.enable = true;

  // get initial position
  geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/ground_truth_to_tf/pose");
  uavPose = msg->pose;

  dest = uavPose;
  dest.position.z += 1;        // start at 1 meter above its starting point

  error.position.x = 0;
  error.position.y = 0;
  error.position.z = 0;
  error.orientation.x = 0;
  error.orientation.y = 0;
  error.orientation.z = 0;

  pub_topic = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  nh.subscribe<geometry_msgs::PoseStamped>("/ground_truth_to_tf/pose", 100, uavController);

  Controller control(nh);

  ros::spin();

  return 0;
}
