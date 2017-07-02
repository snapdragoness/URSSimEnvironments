#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <hector_uav_msgs/EnableMotors.h>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/tokenizer.hpp>

#include <termios.h>

#define ENABLE_MOTORS_SERVICE "/enable_motors"

typedef struct PID
{
  double p, i, d;
} PID;

typedef struct Pose
{
  double x, y, z;
  double yaw;
} Pose;

double quaternionToYaw(const geometry_msgs::QuaternionConstPtr& q);
void uavController(const geometry_msgs::PoseStampedConstPtr& msg, const int uavID);
void uavCommander(ros::Rate rate, const int uavID);
void navigate(const Pose& targetPose, const int delay, const bool orient, const int uavID);
void wait(int milliseconds);
void readLaserScan(const sensor_msgs::LaserScanConstPtr& msg, const int uavID);
void readSonarDownward(const sensor_msgs::RangeConstPtr& msg, const int uavID);
void readSonarUpward(const sensor_msgs::RangeConstPtr& msg, const int uavID);
double getDistance(const Pose& from, const Pose& to);
double getYawDiff(double yaw1, double yaw2);

void dummy();   // only for testing

void initTermios(int echo);
void resetTermios(void);
char getch_(int echo);
char getch(void);
char getche(void);

const unsigned int nUAV = 4;  // the total number of the UAVs
std::string nsUAV[nUAV];      // namespaces of the UAVs

ros::Subscriber poseSub[nUAV];          // subscribers of each UAV's ground truth
ros::Subscriber laserSub[nUAV];         // subscribers of each UAV's laser scanner
ros::Subscriber sonarDownwardSub[nUAV]; // subscribers of each UAV's downward sonar sensor
ros::Subscriber sonarUpwardSub[nUAV];   // subscribers of each UAV's upward sonar sensor

geometry_msgs::Twist cmd[nUAV];   // a twist message to be sent to /cmd_vel

Pose pose[nUAV];        // UAV's position from ground truth
Pose dest[nUAV];        // destinations of the UAVs (used in uavController())
Pose targetDest[nUAV];  // target destinations of the UAVs to be navigated to

/* for PID control */
//PID pidConst = {0.5, 0.0002, 0.00005};  // not good in perfect environment

// read about adjusting PID coefficients at https://oscarliang.com/quadcopter-pid-explained-tuning/
PID pidConst = {0.5, 0.0, 0.0};
Pose error[nUAV];
Pose prevError[nUAV];
Pose proportional[nUAV];
Pose integral[nUAV];
Pose derivation[nUAV];

/* threads */
boost::thread commanderThread[nUAV];
boost::thread navigationThread[nUAV];

/* a mutex should be defined for each critical variable */
boost::mutex mut_cmd[nUAV];
boost::mutex mut_dest[nUAV];
boost::mutex mut_laser[nUAV];
boost::mutex mut_pose[nUAV];
boost::mutex mut_sonarDownward[nUAV];
boost::mutex mut_sonarUpward[nUAV];

/* for laser scan */
sensor_msgs::LaserScan laserScan[nUAV];       // range from 0.1 - 30 meters
const double laserAngularResolution = 0.25;   // a degree for each step
const int laserMeasurementSteps = 1080;       // total scan steps. This indicates the number of elements in laserScan.ranges
const int laserCenterStep = laserMeasurementSteps / 2;  // center front of the laser scanner

/* for sonar scan */
float sonarDownwardRange[nUAV];   // range from 0.03 - 3 meters
float sonarUpwardRange[nUAV];     // range from 0.03 - 3 meters

/* for navigation */
const double maxPositionError = 0.2;      // maximum position error in meter (only non-negative values)
const double maxOrientationError = 1.0;   // maximum orientation error in degree (only non-negative values)
const double distFromObs = 0.1;           // minimum distance to be kept from obstacles (only non-negative values)
                                          // the value has to be within all navigating sensors' ranges
const double quadrotorLength = 1.0;       // suppose the quadrotors have square shape
const double safetyMargin = quadrotorLength / 2.0 + distFromObs;  // used in navigate() to make sure there is enough space
                                                                  // for the quadrotor at the target destination

/* for keyboard teleop */
unsigned int activeID = 0;  // the ID of the UAV that receive gamepad-like control from keyboard
                            // possible values: 0 to nUAV-1
double moveStep = 0.5;
double rotateStep = 0.5;

int main(int argc, char **argv)
{
  /* initialize ROS, and sets up a node */
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

  if (nUAV == 0) {
    ROS_ERROR("No UAV to control");
    exit(EXIT_FAILURE);
  }

  /* initialize various variables */
  for (unsigned int i = 0; i < nUAV; i++)
  {
    // set a namespace for each uav
    nsUAV[i] = "/uav" + std::to_string(i);

    // create a service client
    ros::ServiceClient serviceClient = nh->serviceClient<hector_uav_msgs::EnableMotors>(nsUAV[i] + ENABLE_MOTORS_SERVICE);
    hector_uav_msgs::EnableMotors enable_motors_srv;
    enable_motors_srv.request.enable = true;
    if (serviceClient.call(enable_motors_srv))
    {
      ROS_INFO("%s - %s\n", nh->resolveName(nsUAV[i] + ENABLE_MOTORS_SERVICE).c_str(),
               enable_motors_srv.response.success ? "successful" : "failed");

      // get initial position
      geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
          nsUAV[i] + "/ground_truth_to_tf/pose");

      // start at 1 meter above its starting point while preserve the orientation
      dest[i].x = msg->pose.position.x;
      dest[i].y = msg->pose.position.y;
      dest[i].z = msg->pose.position.z + 1.0;
      dest[i].yaw = quaternionToYaw(boost::make_shared<geometry_msgs::Quaternion>(msg->pose.orientation));

      error[i].x = 0;
      error[i].y = 0;
      error[i].z = 0;
      error[i].yaw = 0;

      poseSub[i] = nh->subscribe<geometry_msgs::PoseStamped>(nsUAV[i] + "/ground_truth_to_tf/pose", 10,
                                                             boost::bind(uavController, _1, i));
      laserSub[i] = nh->subscribe<sensor_msgs::LaserScan>(nsUAV[i] + "/scan", 10,
                                                          boost::bind(readLaserScan, _1, i));
      sonarDownwardSub[i] = nh->subscribe<sensor_msgs::Range>(nsUAV[i] + "/sonar_downward", 10,
                                                              boost::bind(readSonarDownward, _1, i));
      sonarUpwardSub[i] = nh->subscribe<sensor_msgs::Range>(nsUAV[i] + "/sonar_upward", 10,
                                                            boost::bind(readSonarUpward, _1, i));
    }
    else
    {
      ROS_ERROR("Failed to call %s", nh->resolveName(nsUAV[i] + ENABLE_MOTORS_SERVICE).c_str());
    }
  }

  std::cout << "Total number of UAVs: " << nUAV
            << " [ID: " << ((nUAV == 1)? "0": "0-" + std::to_string(nUAV - 1)) << "]" << std::endl;
  std::cout << "Active ID: " << activeID << std::endl;

  /* spawn other threads (must be done after initialization) */
  for (unsigned int i = 0; i < nUAV; i++) {
    commanderThread[i] = boost::thread(uavCommander, 10, i);
  }

//  boost::thread dummyThread = boost::thread(dummy);   // only for testing

  /* read and evaluate keyboard input */
  ros::Rate rate(10);
  while (ros::ok())
  {
    char key = getch();
    switch (key)
    {
      case 'w':
      case 'W':
        mut_dest[activeID].lock();
        dest[activeID].x += moveStep;
        mut_dest[activeID].unlock();
        break;
      case 's':
      case 'S':
        mut_dest[activeID].lock();
        dest[activeID].x -= moveStep;
        mut_dest[activeID].unlock();
        break;
      case 'a':
      case 'A':
        mut_dest[activeID].lock();
        dest[activeID].y += moveStep;
        mut_dest[activeID].unlock();
        break;
      case 'd':
      case 'D':
        mut_dest[activeID].lock();
        dest[activeID].y -= moveStep;
        mut_dest[activeID].unlock();
        break;
      case 'r':
      case 'R':
        mut_dest[activeID].lock();
        dest[activeID].z += moveStep;
        mut_dest[activeID].unlock();
        break;
      case 'f':
      case 'F':
        mut_dest[activeID].lock();
        dest[activeID].z -= moveStep;
        mut_dest[activeID].unlock();
        break;
      case 'q':
      case 'Q':
        mut_dest[activeID].lock();
        dest[activeID].yaw += rotateStep;
        mut_dest[activeID].unlock();
        break;
      case 'e':
      case 'E':
        mut_dest[activeID].lock();
        dest[activeID].yaw -= rotateStep;
        mut_dest[activeID].unlock();
        break;
      case 't':
      case 'T':
        moveStep += 0.1;
        std::cout << "move step = " << std::setprecision(1) << std::fixed << moveStep << std::endl;
        break;
      case 'g':
      case 'G':
        moveStep -= 0.1;
        std::cout << "move step = " << std::setprecision(1) << std::fixed << moveStep << std::endl;
        break;
      case 'y':
      case 'Y':
        rotateStep += 0.1;
        std::cout << "rotate step = " << std::setprecision(1) << std::fixed << rotateStep << std::endl;
        break;
      case 'h':
      case 'H':
        rotateStep -= 0.1;
        std::cout << "rotate step = " << std::setprecision(1) << std::fixed << rotateStep << std::endl;
        break;
      case ':':
        std::cout << ": ";
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
            if (tokens.size() == 2 && !tokens[0].compare("land"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                mut_dest[uavID].lock();
                dest[uavID].z = 0.5;   // TODO: should actually check from the downward sonar sensor
                mut_dest[uavID].unlock();
              }
            }
            else if (tokens.size() == 5 && !tokens[0].compare("move"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                navigationThread[uavID].interrupt();  // interrupt the old thread, if any

                mut_pose[uavID].lock();
                targetDest[uavID].x = pose[uavID].x + std::stod(tokens[2]);
                targetDest[uavID].y = pose[uavID].y + std::stod(tokens[3]);
                targetDest[uavID].z = pose[uavID].z + std::stod(tokens[4]);
                targetDest[uavID].yaw = pose[uavID].yaw;
                mut_pose[uavID].unlock();

                navigationThread[uavID] = boost::thread(navigate, targetDest[uavID], 100, false, uavID);
              }
            }
            else if (tokens.size() == 5 && !tokens[0].compare("goto"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                navigationThread[uavID].interrupt();  // interrupt the old thread, if any

                targetDest[uavID].x = std::stod(tokens[2]);
                targetDest[uavID].y = std::stod(tokens[3]);
                targetDest[uavID].z = std::stod(tokens[4]);
                mut_pose[uavID].lock();
                targetDest[uavID].yaw = pose[uavID].yaw;
                mut_pose[uavID].unlock();

                navigationThread[uavID] = boost::thread(navigate, targetDest[uavID], 100, false, uavID);
              }
            }
            else if (tokens.size() == 3 && !tokens[0].compare("rotate"))
            {
              int uavID = std::stoi(tokens[1]);
              double degree = std::stod(tokens[2]);
              mut_dest[uavID].lock();
              dest[uavID].yaw = degree * M_PI / 180.0;
              mut_dest[uavID].unlock();
            }
            else if (tokens.size() == 2 && !tokens[0].compare("cancel"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                navigationThread[uavID].interrupt();
              }
            }
            else if (tokens.size() == 1)
            {
              int uavID = std::stoi(tokens[0]);
              if (uavID >= 0 && uavID < nUAV)
              {
                activeID = uavID;
                std::cout << "Active ID: " << activeID << std::endl;
              }
            }
          }
          catch (...)
          {
            std::cout << "Error parsing some parameters" << std::endl;
          }
        }
        break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  /* clean up the subscribers */
  for (unsigned int i = 0; i < nUAV; i++) {
    poseSub[i].shutdown();
    laserSub[i].shutdown();
    sonarDownwardSub[i].shutdown();
    sonarUpwardSub[i].shutdown();
  }
}

double quaternionToYaw(const geometry_msgs::QuaternionConstPtr& q)
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

// PID Control
void uavController(const geometry_msgs::PoseStampedConstPtr& msg, const int uavID)
{
  double yaw = quaternionToYaw(boost::make_shared<geometry_msgs::Quaternion>(msg->pose.orientation));

  mut_pose[uavID].lock();
  pose[uavID].x = msg->pose.position.x;
  pose[uavID].y = msg->pose.position.y;
  pose[uavID].z = msg->pose.position.z;
  pose[uavID].yaw = yaw;
  mut_pose[uavID].unlock();

  prevError[uavID].x = error[uavID].x;
  prevError[uavID].y = error[uavID].y;
  prevError[uavID].z = error[uavID].z;
  prevError[uavID].yaw = error[uavID].yaw;

  mut_dest[uavID].lock();
  error[uavID].x = dest[uavID].x - msg->pose.position.x;
  error[uavID].y = dest[uavID].y - msg->pose.position.y;
  error[uavID].z = dest[uavID].z - msg->pose.position.z;
  error[uavID].yaw = dest[uavID].yaw - yaw;
  mut_dest[uavID].unlock();

  if (error[uavID].yaw < -M_PI) {
    error[uavID].yaw += M_PI + M_PI;
  } else if (error[uavID].yaw > M_PI) {
    error[uavID].yaw -= M_PI + M_PI;
  }

  proportional[uavID].x = pidConst.p * error[uavID].x;
  proportional[uavID].y = pidConst.p * error[uavID].y;
  proportional[uavID].z = pidConst.p * error[uavID].z;
  integral[uavID].x += pidConst.i * error[uavID].x;
  integral[uavID].y += pidConst.i * error[uavID].y;
  integral[uavID].z += pidConst.i * error[uavID].z;
  derivation[uavID].x = pidConst.d * (error[uavID].x - prevError[uavID].x);
  derivation[uavID].y = pidConst.d * (error[uavID].y - prevError[uavID].y);
  derivation[uavID].z = pidConst.d * (error[uavID].z - prevError[uavID].z);

  double x, y, z;
  x = proportional[uavID].x + integral[uavID].x + derivation[uavID].x;
  y = proportional[uavID].y + integral[uavID].y + derivation[uavID].y;
  z = proportional[uavID].z + integral[uavID].z + derivation[uavID].z;

  double rx, ry;
  rx = x * std::cos(-1 * yaw) - y * std::sin(-1 * yaw);
  ry = x * std::sin(-1 * yaw) + y * std::cos(-1 * yaw);

  mut_cmd[uavID].lock();
  cmd[uavID].linear.x = rx;
  cmd[uavID].linear.y = ry;
  cmd[uavID].linear.z = z;
  cmd[uavID].angular.z = 2.0 * error[uavID].yaw;
  mut_cmd[uavID].unlock();
}

void uavCommander(ros::Rate rate, const int uavID)
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::Publisher cmdPub;
  cmdPub = nh->advertise<geometry_msgs::Twist>(nsUAV[uavID] + "/cmd_vel", 10, false);

  while (ros::ok())
  {
    mut_cmd[uavID].lock();
    cmdPub.publish(cmd[uavID]);
    mut_cmd[uavID].unlock();
    ros::spinOnce();
    rate.sleep();
  }

  /* clean up the publisher */
  cmdPub.shutdown();
}

void dummy()
{
  while(true)
  {
    Pose currentPose;
    mut_pose[0].lock();
    currentPose.x = pose[0].x;
    currentPose.y = pose[0].y;
    currentPose.z = pose[0].z;
    currentPose.yaw = pose[0].yaw;
    mut_pose[0].unlock();

    std::cout << "pose[0].x: " << currentPose.x << std::endl;
    std::cout << "pose[0].y: " << currentPose.y << std::endl;
    std::cout << "pose[0].z: " << currentPose.z << std::endl;
    std::cout << "pose[0].yaw: " << currentPose.yaw * 180.0 / M_PI << std::endl;
    mut_dest[0].lock();
    std::cout << "diff = " << getDistance(currentPose, dest[0]) << std::endl;
    mut_dest[0].unlock();
    wait(10);
  }
}

void navigate(const Pose& targetPose, const int delay, const bool orient, const int uavID)
{
  std::cout << "A navigation thread for UAV" << uavID << " to ["
      << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has started" << std::endl;

  try
  {
    Pose partialTargetPose;
    partialTargetPose.x = targetPose.x;
    partialTargetPose.y = targetPose.y;
    partialTargetPose.z = targetPose.z;

    int reachCount = 0;

    while (true)
    {
      Pose currentPose;
      mut_pose[uavID].lock();
      currentPose.x = pose[uavID].x;
      currentPose.y = pose[uavID].y;
      currentPose.z = pose[uavID].z;
      currentPose.yaw = pose[uavID].yaw;
      mut_pose[uavID].unlock();

      // if the UAV reaches the target destination
      if (getDistance(currentPose, targetPose) <= maxPositionError)
      {
        if (!orient || getYawDiff(currentPose.yaw, targetPose.yaw) <= maxOrientationError * M_PI / 180.0) {
          break;
        }
        else
        {
          mut_dest[uavID].lock();
          dest[uavID].yaw = targetPose.yaw;
          mut_dest[uavID].unlock();
        }
      }
      else  // otherwise, begin navigating
      {
        // update the partial target destination if the UAV has reached it
        if (getDistance(currentPose, partialTargetPose) <= maxPositionError)
        {
          if (reachCount >= 10)
          {
            partialTargetPose.x = targetPose.x;
            partialTargetPose.y = targetPose.y;
            partialTargetPose.z = targetPose.z;
          }
          else
          {
            reachCount++;
          }
        }
        else
        {
          reachCount = 0;
        }

        double planarDistToTarget = std::sqrt(
            (partialTargetPose.x - currentPose.x) * (partialTargetPose.x - currentPose.x)
            + (partialTargetPose.y - currentPose.y) * (partialTargetPose.y - currentPose.y));
        double yawHeaded;

        if (planarDistToTarget > maxPositionError)  // if the quadrotor needs to move horizontally
        {
          yawHeaded = std::atan2(partialTargetPose.y - currentPose.y, partialTargetPose.x - currentPose.x) - std::atan2(0, 1);
          if (yawHeaded < 0.0)
          {
            yawHeaded += M_PI + M_PI;
          }

          mut_dest[uavID].lock();
          dest[uavID].yaw = yawHeaded;
          mut_dest[uavID].unlock();

//          std::cout << "yaw diff = " << getYawDiff(currentPose.yaw, yawHeaded) * 180.0 / M_PI << std::endl;

          // if the quadrotor has aligned itself to the partial target destination
          if (getYawDiff(currentPose.yaw, yawHeaded) <= M_PI / 180.0)  // allow 1.0 degree of error
          {
            bool obstructed = false;

            double panDegree = std::atan2(safetyMargin, planarDistToTarget) * 180.0 / M_PI;
            int panSteps = ((panDegree / laserAngularResolution) + 1 > 150)? 150 : (panDegree / laserAngularResolution) + 1;

            mut_laser[uavID].lock();
            if (!laserScan[uavID].ranges.empty())
            {
              if (laserScan[uavID].ranges[laserCenterStep] < planarDistToTarget + safetyMargin)
              {
                obstructed = true;
              }
              else
              {
                double angle = laserAngularResolution;
                for (int i = 1; i <= panSteps; i++)
                {
                  double lengthRequired = (planarDistToTarget + safetyMargin) / std::cos(angle * M_PI / 180.0);
                  if ((laserScan[uavID].ranges[laserCenterStep - i] < lengthRequired)
                      || (laserScan[uavID].ranges[laserCenterStep + i] < lengthRequired))
                  {
                    obstructed = true;
                    break;
                  }
                  angle += laserAngularResolution;
                }
              }
            }
            mut_laser[uavID].unlock();

            if (obstructed)   // if the path is blocked, we need to find a new partial target
            {
              std::cout << "path blocked" << std::endl;
              partialTargetPose.x = currentPose.x;
              partialTargetPose.y = currentPose.y;
              partialTargetPose.z = currentPose.z + 0.5;    // TODO: to go up, better check the sonar sensor too
            }

            mut_dest[uavID].lock();
            dest[uavID].x = partialTargetPose.x;
            dest[uavID].y = partialTargetPose.y;
            dest[uavID].z = partialTargetPose.z;
            mut_dest[uavID].unlock();
          }
          else    // otherwise, don't move, just rotate
          {
            if (getDistance(currentPose, partialTargetPose) > maxPositionError)
            {
              std::cout << "rotating" << std::endl;
              partialTargetPose.x = currentPose.x;
              partialTargetPose.y = currentPose.y;
              partialTargetPose.z = currentPose.z;
            }
          }
        }
        else    // if the quadrotor only needs to move vertically
        {
          mut_dest[uavID].lock();
          dest[uavID].z = partialTargetPose.z;
          mut_dest[uavID].unlock();

          float sonarRange;
          if (partialTargetPose.z < currentPose.z)  // going downward
          {
            mut_sonarDownward[uavID].lock();
            sonarRange = sonarDownwardRange[uavID];
            mut_sonarDownward[uavID].unlock();
          }
          else  // going upward
          {
            mut_sonarUpward[uavID].lock();
            sonarRange = sonarUpwardRange[uavID];
            mut_sonarUpward[uavID].unlock();
          }

          if (sonarRange <= 1.0)  // should actually be distFromObs
          {
            mut_dest[uavID].lock();
            dest[uavID].z = currentPose.z;
            mut_dest[uavID].unlock();

            std::cout << "Navigation failed" << std::endl;
            break;
          }
        }
      }

      wait(delay);  // 'delay' is in milliseconds
    }
  }
  catch (boost::thread_interrupted&)
  {
    std::cout << "A navigation thread for UAV" << uavID << " to ["
        << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has been interrupted" << std::endl;
  }
  std::cout << "A navigation thread for UAV" << uavID << " to ["
      << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has terminated" << std::endl;
}

void wait(int milliseconds)
{
  boost::this_thread::sleep_for(boost::chrono::milliseconds{milliseconds});
}


void readLaserScan(const sensor_msgs::LaserScanConstPtr& msg, const int uavID)
{
  mut_laser[uavID].lock();
  laserScan[uavID] = *msg;
  mut_laser[uavID].unlock();
}

void readSonarDownward(const sensor_msgs::RangeConstPtr& msg, const int uavID)
{
  mut_sonarDownward[uavID].lock();
  sonarDownwardRange[uavID] = msg->range;
  mut_sonarDownward[uavID].unlock();
}

void readSonarUpward(const sensor_msgs::RangeConstPtr& msg, const int uavID)
{
  mut_sonarUpward[uavID].lock();
  sonarUpwardRange[uavID] = msg->range;
  mut_sonarUpward[uavID].unlock();
}

double getDistance(const Pose& from, const Pose& to)
{
  return std::sqrt(
      (to.x - from.x) * (to.x - from.x)
      + (to.y - from.y) * (to.y - from.y)
      + (to.z - from.z) * (to.z - from.z));
}

double getYawDiff(double yaw1, double yaw2)   // yaw1 and yaw2 are in radian
{
  double diff = std::fabs(yaw1 - yaw2);
  return (diff > M_PI)? M_PI + M_PI - diff: diff;
}

//////////////////////////////////////////////////////////////////////////////
struct termios old_t, new_t;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
  tcgetattr(0, &old_t); /* grab old terminal i/o settings */
  new_t = old_t; /* make new settings same as old settings */
  new_t.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_t.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_t); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &old_t);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void)
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void)
{
  return getch_(1);
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
