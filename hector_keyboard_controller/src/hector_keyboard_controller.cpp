#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>

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

enum class ControlMode
{
  disabled = 0, manual, autopilot
};

// This may be used to organize more mutexs (in the future)
enum class CriticalRegion
{
};

Euler quaternionToEuler(geometry_msgs::Quaternion q);
void uavController(const geometry_msgs::PoseStamped::ConstPtr& msg, const int uavID);
void uavCommander(ros::Rate rate);
void printControlStatus();

const unsigned int nUAV = 4;
const unsigned int nWaypoint = 4;

std::string nsUAV[nUAV];
ControlMode controlMode[nUAV] = {ControlMode::disabled};

// waypoints for 4 UAVs
geometry_msgs::Pose waypoint[nUAV][nWaypoint];

// manually-set destinations for nUAV
geometry_msgs::Pose dest[nUAV];

bool destReached[nUAV] = {false};

PID pidConst = {0.5, 0.0002, 0.00005};
const double tolerance = 0.5;

int waypointNumber = 0;

geometry_msgs::Pose uavPose[nUAV];     // real location of the UAVs

/* for PID control */
geometry_msgs::Pose error[nUAV];
geometry_msgs::Pose prevError[nUAV];
geometry_msgs::Pose proportional[nUAV];
geometry_msgs::Pose integral[nUAV];
geometry_msgs::Pose derivation[nUAV];

// message to be send to drone
geometry_msgs::Twist twist[nUAV];

boost::mutex mut[nUAV];

int main(int argc, char **argv)
{
  // initializes ROS, and sets up a node
  ros::init(argc, argv, "hector_keyboard_controller");

  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

  // create subscribers
  ros::Subscriber uavSub[nUAV];
  ros::Subscriber keyboardInputSubscriber;// = np->subscribe<std_msgs::String>("keyboard_input", 10, updateSettings);

  // create a service client
  ros::ServiceClient client;
  hector_uav_msgs::EnableMotors enable_motors_srv;

  /* Initialize */
  for (int i = 0; i < nUAV; i++)
  {
    nsUAV[i] = "/uav" + std::to_string(i);

    client = nh->serviceClient<hector_uav_msgs::EnableMotors>(nsUAV[i] + ENABLE_MOTORS_SERVICE);
    enable_motors_srv.request.enable = true;
    if (client.call(enable_motors_srv))
    {
      ROS_INFO("%s - %s\n", nh->resolveName(nsUAV[i] + ENABLE_MOTORS_SERVICE).c_str(),
               enable_motors_srv.response.success ? "successful" : "failed");

      // get initial position
      geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
          nsUAV[i] + "/ground_truth_to_tf/pose");

      uavPose[i] = msg->pose;
      destReached[i] = false;
      controlMode[i] = ControlMode::manual;     // should always start in manual mode

      if (controlMode[i] == ControlMode::manual)
      {
        dest[i] = uavPose[i];
        dest[i].position.z += 1;        // start at 1 meter above its starting point
      }
      else if (controlMode[i] == ControlMode::autopilot)
      {
        dest[i] = waypoint[i][waypointNumber];
      }

      error[i].position.x = 0;
      error[i].position.y = 0;
      error[i].position.z = 0;
      error[i].orientation.x = 0;
      error[i].orientation.y = 0;
      error[i].orientation.z = 0;

      uavSub[i] = nh->subscribe<geometry_msgs::PoseStamped>(nsUAV[i] + "/ground_truth_to_tf/pose", 100,
                                                                      boost::bind(uavController, _1, i));
    }
    else
    {
      ROS_ERROR("Failed to call %s", nh->resolveName(nsUAV[i] + ENABLE_MOTORS_SERVICE).c_str());
    }
  }

  if (nWaypoint > 0)
  {
    waypoint[0][0].position.x = 3; waypoint[0][0].position.y = -3; waypoint[0][0].position.z = 3;
    waypoint[0][1].position.x = 3; waypoint[0][1].position.y = 3; waypoint[0][1].position.z = 4;
    waypoint[0][2].position.x = -3; waypoint[0][2].position.y = 3; waypoint[0][2].position.z = 3;
    waypoint[0][3].position.x = -3; waypoint[0][3].position.y = -3; waypoint[0][3].position.z = 2;
  }
  if (nWaypoint > 1)
  {
    waypoint[1][0].position.x = 3; waypoint[1][0].position.y = 3; waypoint[1][0].position.z = 4;
    waypoint[1][1].position.x = -3; waypoint[1][1].position.y = 3; waypoint[1][1].position.z = 3;
    waypoint[1][2].position.x = -3; waypoint[1][2].position.y = -3; waypoint[1][2].position.z = 2;
    waypoint[1][3].position.x = 3; waypoint[1][3].position.y = -3; waypoint[1][3].position.z = 3;
  }
  if (nWaypoint > 2)
  {
    waypoint[2][0].position.x = -3; waypoint[2][0].position.y = 3; waypoint[2][0].position.z = 3;
    waypoint[2][1].position.x = -3; waypoint[2][1].position.y = -3; waypoint[2][1].position.z = 2;
    waypoint[2][2].position.x = 3; waypoint[2][2].position.y = -3; waypoint[2][2].position.z = 3;
    waypoint[2][3].position.x = 3; waypoint[2][3].position.y = 3; waypoint[2][3].position.z = 4;
  }
  if (nWaypoint > 3)
  {
    waypoint[3][0].position.x = -3; waypoint[3][0].position.y = -3; waypoint[3][0].position.z = 2;
    waypoint[3][1].position.x = 3; waypoint[3][1].position.y = -3; waypoint[3][1].position.z = 3;
    waypoint[3][2].position.x = 3; waypoint[3][2].position.y = 3; waypoint[3][2].position.z = 4;
    waypoint[3][3].position.x = -3; waypoint[3][3].position.y = 3; waypoint[3][3].position.z = 3;
  }

  printControlStatus();

  /* spawn other threads (must be done after initialization) */
  boost::thread commanderThread(uavCommander, 10);

  /* get and evaluate keyboard input */
  ros::Rate rate(10);
  while (ros::ok())
  {
    std::cout << ">> ";
    std::string keyboardInput;
    std::getline(std::cin, keyboardInput);

    // remove trailing end-line characters
    while (keyboardInput.size()
        && (keyboardInput[keyboardInput.size() - 1] == '\n' || keyboardInput[keyboardInput.size() - 1] == '\r'))
    {
      keyboardInput = keyboardInput.substr(0, keyboardInput.size() - 1);
    }

    if (keyboardInput.size())
    {
      // break the input into tokens
      std::vector<std::string> tokens;
      boost::char_separator<char> sep {" "};
      boost::tokenizer<boost::char_separator<char>> tok {keyboardInput, sep};
      for (const auto &token : tok)
      {
        tokens.push_back(token);
      }

      // check the input
      try
      {
        if (!tokens[0].compare("disable"))
        {
          int uavID = std::stoi(tokens[1]);
          if (uavID >= 0 && uavID < nUAV)
          {
            mut[uavID].lock();
            controlMode[uavID] = ControlMode::disabled;
            mut[uavID].unlock();
            printControlStatus();
          }
        }
        else if (!tokens[0].compare("manual"))
        {
          int uavID = std::stoi(tokens[1]);
          if (uavID >= 0 && uavID < nUAV)
          {
            mut[uavID].lock();
            destReached[uavID] = false;
            controlMode[uavID] = ControlMode::manual;
            mut[uavID].unlock();
            printControlStatus();
          }
        }
        else if (!tokens[0].compare("auto"))
        {
          int uavID = std::stoi(tokens[1]);
          if (uavID >= 0 && uavID < nUAV)
          {
            mut[uavID].lock();
            destReached[uavID] = false;
            controlMode[uavID] = ControlMode::autopilot;
            dest[uavID] = waypoint[uavID][waypointNumber];
            mut[uavID].unlock();
            printControlStatus();
          }
        }
        else if (!tokens[0].compare("land"))
        {
          int uavID = std::stoi(tokens[1]);
          if (uavID >= 0 && uavID < nUAV)
          {
            mut[uavID].lock();
            controlMode[uavID] = ControlMode::manual;
            destReached[uavID] = false;
            dest[uavID].position.z = 0.5;
            mut[uavID].unlock();
            printControlStatus();
          }
        }
        else if (!tokens[0].compare("move"))
        {
          for (int i = 0; i < nUAV; i++)
          {
            mut[i].lock();
            if (controlMode[i] == ControlMode::manual)
            {
              destReached[i] = false;
              dest[i].position.x += std::stod(tokens[1]);
              dest[i].position.y += std::stod(tokens[2]);
              dest[i].position.z += std::stod(tokens[3]);
            }
            mut[i].unlock();
          }
        }
        else if (!tokens[0].compare("goto"))
        {
          for (int i = 0; i < nUAV; i++)
          {
            mut[i].lock();
            if (controlMode[i] == ControlMode::manual)
            {
              destReached[i] = false;
              dest[i].position.x = std::stod(tokens[1]);
              dest[i].position.y = std::stod(tokens[2]);
              dest[i].position.z = std::stod(tokens[3]);
            }
            mut[i].unlock();
          }
        }
        else
        {
          std::cout << "Invalid command" << std::endl;
        }
      }
      catch (...)
      {
        std::cout << "Error parsing some parameters" << std::endl;
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}

Euler quaternionToEuler(geometry_msgs::Quaternion q)
{
  Euler euler;

  double ysqr = q.y * q.y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (q.w * q.x + q.y * q.z);
  double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
  euler.roll = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (q.w * q.y - q.z * q.x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  euler.pitch = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q.w * q.z + q.x * q.y);
  double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
  euler.yaw = std::atan2(t3, t4);

  return euler;
}

// PID Control
void uavController(const geometry_msgs::PoseStamped::ConstPtr& msg, int uavID)
{
  Euler euler = quaternionToEuler(msg->pose.orientation);

  uavPose[uavID].position.x = msg->pose.position.x;
  uavPose[uavID].position.y = msg->pose.position.y;
  uavPose[uavID].position.z = msg->pose.position.z;
//  uavPose[uavID].orientation.z = msg->pose.orientation.z;

  prevError[uavID].position.x = error[uavID].position.x;
  prevError[uavID].position.y = error[uavID].position.y;
  prevError[uavID].position.z = error[uavID].position.z;
//  prevError[uavID].orientation.z = error[uavID].orientation.z;
  mut[uavID].lock();
  error[uavID].position.x = dest[uavID].position.x - uavPose[uavID].position.x;
  error[uavID].position.y = dest[uavID].position.y - uavPose[uavID].position.y;
  error[uavID].position.z = dest[uavID].position.z - uavPose[uavID].position.z;
//  error[uavID].orientation.z = 0 - uavPose[uavID].orientation.z;
  mut[uavID].unlock();

  proportional[uavID].position.x = pidConst.p * error[uavID].position.x;
  proportional[uavID].position.y = pidConst.p * error[uavID].position.y;
  proportional[uavID].position.z = pidConst.p * error[uavID].position.z;
//  proportional[uavID].orientation.z = pidConst.p * error[uavID].orientation.z;
  integral[uavID].position.x += pidConst.i * error[uavID].position.x;
  integral[uavID].position.y += pidConst.i * error[uavID].position.y;
  integral[uavID].position.z += pidConst.i * error[uavID].position.z;
//  integral[uavID].orientation.z += pidConst.i * error[uavID].orientation.z;
  derivation[uavID].position.x = pidConst.d * (error[uavID].position.x - prevError[uavID].position.x);
  derivation[uavID].position.y = pidConst.d * (error[uavID].position.y - prevError[uavID].position.y);
  derivation[uavID].position.z = pidConst.d * (error[uavID].position.z - prevError[uavID].position.z);
//  derivation[uavID].orientation.z = pidConst.d * (error[uavID].orientation.z - prevError[uavID].orientation.z);

  double x, y, z;
  x = proportional[uavID].position.x + integral[uavID].position.x + derivation[uavID].position.x;
  y = proportional[uavID].position.y + integral[uavID].position.y + derivation[uavID].position.y;
  z = proportional[uavID].position.z + integral[uavID].position.z + derivation[uavID].position.z;
//  action.orientation.z = proportional[uavID].orientation.z + integral[uavID].orientation.z + derivation[uavID].orientation.z;
  double rx, ry;
  rx = x * cos(-1 * euler.yaw) - y * sin(-1 * euler.yaw);
  ry = x * sin(-1 * euler.yaw) + y * cos(-1 * euler.yaw);

  mut[uavID].lock();
  twist[uavID].linear.x = rx;
  twist[uavID].linear.y = ry;
  twist[uavID].linear.z = z;
  mut[uavID].unlock();

  if ((std::fabs(error[uavID].position.x) < tolerance) && (std::fabs(error[uavID].position.y) < tolerance)
      && (std::fabs(error[uavID].position.z) < tolerance))
  {
    mut[uavID].lock();
    destReached[uavID] = true;
    mut[uavID].unlock();
  }
}

void uavCommander(ros::Rate rate)
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::Publisher uavPub[nUAV];

  for (int i = 0; i < nUAV; i++)
  {
    uavPub[i] = nh->advertise<geometry_msgs::Twist>(nsUAV[i] + "/cmd_vel", 100, false);
  }

  while (ros::ok())
  {
    bool allAutopilotReachedDest = true;
    int nAutopilotUAV = 0;
    for (int i = 0; i < nUAV; i++)
    {
      mut[i].lock();
      if (controlMode[i] != ControlMode::disabled)
      {
        uavPub[i].publish(twist[i]);

        if (controlMode[i] == ControlMode::autopilot)
        {
          allAutopilotReachedDest = allAutopilotReachedDest && destReached[i];
          nAutopilotUAV++;
        }
      }
      mut[i].unlock();
    }

    if (allAutopilotReachedDest && nAutopilotUAV)
    {
      if (++waypointNumber >= nWaypoint)
      {
        waypointNumber = 0;
      }
      for (int i = 0; i < nUAV; i++)
      {
        mut[i].lock();
        if (controlMode[i] == ControlMode::autopilot)
        {
          destReached[i] = false;
          dest[i] = waypoint[i][waypointNumber];
        }
        mut[i].unlock();
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void printControlStatus()
{
  for (int i = 0; i < nUAV; i++)
  {
    std::cout << "UAV" << i << " - ";
    mut[i].lock();
    switch (controlMode[i])
    {
      case ControlMode::disabled:
        std::cout << "disabled";
        break;
      case ControlMode::manual:
        std::cout << "manual";
        break;
      case ControlMode::autopilot:
        std::cout << "autopilot";
        break;
    }
    mut[i].unlock();
    if (i < nUAV - 1)
    {
      std::cout << "; ";
    }
    else
    {
      std::cout << std::endl;
    }
  }
}
