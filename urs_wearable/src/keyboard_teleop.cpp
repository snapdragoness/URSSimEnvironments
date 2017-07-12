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

/*
  Legal indexes (that do not get blocked by the quadrotor's legs) of the laser scan are:
  23-335, 381-694, 737-1065

  which could be trimmed down to:
  30-330, 390-690, 750-1050

  Measurement Steps = 1080  ;   Total Degree = 270 degree
           540              ;             0 deg
            |               ;               |
    180 --------- 900       ;   -90 deg --------- 90 deg
          /   \             ;             /   \
         0    1080          ;       -135 deg   135 deg
*/

/* for sonar scan */
float sonarDownwardRange[nUAV];   // range from 0.03 - 3 meters
float sonarUpwardRange[nUAV];     // range from 0.03 - 3 meters
const float sonarMaxRange = 3.0;        // maximum value possible from sonar sensors

/* for navigation */
const double maxPositionError = 0.1;      // maximum position error in meter (only non-negative values)
const double maxOrientationError = 1.0;   // maximum orientation error in degree (only non-negative values)
const double distFromObs = 0.2;           // minimum distance to be kept from horizontal obstacles (only non-negative values)
const double distFromObsVertical = 1.0;   // minimum distance to be kept from vertical obstacles (only non-negative values)
                                          // should be at least 1.0. Otherwise, it might not be able to brake in time before crashing
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
                                       // and disable motors
                                       // TODO: write takeoff function
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

                navigationThread[uavID] = boost::thread(navigate, targetDest[uavID], 10, false, uavID);
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

                navigationThread[uavID] = boost::thread(navigate, targetDest[uavID], 10, false, uavID);
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

    bool hasObstructedCenter = false;

    // (int)(std::atan2(0.7, 2.0) * 180.0 / M_PI / 0.25) + 1 = 78
    // (int)(std::atan2(0.5, 1.0) * 180.0 / M_PI / 0.25) + 1 = 57
    // (int)(std::atan2(0.7, 1.0) * 180.0 / M_PI / 0.25) + 1 = 140
    double sideStep = (int)(std::atan2(safetyMargin, 2.0) * 180.0 / M_PI / laserAngularResolution) + 1;

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
          if (hasObstructedCenter)
          {
            partialTargetPose.x = (targetPose.x + currentPose.x) / 2;
            partialTargetPose.y = (targetPose.y + currentPose.y) / 2;
            partialTargetPose.z = currentPose.z + 0.5;
            hasObstructedCenter = false;
          }
          else
          {
            partialTargetPose.x = targetPose.x;
            partialTargetPose.y = targetPose.y;
            partialTargetPose.z = targetPose.z;
          }
            std::cout << uavID << "RE) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
        }

        double vx = partialTargetPose.x - currentPose.x;
        double vy = partialTargetPose.y - currentPose.y;
        double planarDistToTarget = std::sqrt(vx * vx + vy * vy);
        double yawHeaded = std::atan2(vy, vx);
        if (yawHeaded < 0.0)
        {
          yawHeaded += M_PI + M_PI;
        }

        // rotate only if it needs to move horizontally
        if (planarDistToTarget > maxPositionError)
        {
          mut_dest[uavID].lock();
          dest[uavID].yaw = yawHeaded;
          mut_dest[uavID].unlock();
        }

        // if the quadrotor has aligned itself to the partial target destination
        if (getYawDiff(currentPose.yaw, yawHeaded) <= M_PI / 180.0  // allow 1.0 degree of error
            || planarDistToTarget <= maxPositionError)              // or only needs to fly vertically
        {
          if (planarDistToTarget > maxPositionError)  // if needs to move horizontally
          {
            double panDegree = std::atan2(safetyMargin, planarDistToTarget) * 180.0 / M_PI;
            int panStep = (int)(panDegree / laserAngularResolution) + 1;

            // legal range: 390 <- 540 -> 690. The maximum steps from 540 to both side are 150
            int panStepTrimmed = (panStep > 150)? 150: panStep;

            // TODO: In the case that panSteps > 150,
            // there might be a problem if the quadrotor is asked to go through a passage than it can't squeeze through.

            /* check the laser scanner */
            mut_laser[uavID].lock();
            if (!laserScan[uavID].ranges.empty())
            {
              bool obstructedCenter = false;
              bool obstructedLeft = false;
              bool obstructedRight = false;
              double obstructedCenterDist = DBL_MAX;
              double obstructedLeftDist = DBL_MAX;
              double obstructedRightDist = DBL_MAX;

              if (laserScan[uavID].ranges[laserCenterStep] < planarDistToTarget + safetyMargin)
              {
                obstructedCenter = true;
                obstructedCenterDist = laserScan[uavID].ranges[laserCenterStep];
              }
              double upperLimit = (planarDistToTarget + safetyMargin) / std::cos(panStepTrimmed * laserAngularResolution * M_PI / 180.0);
              if (laserScan[uavID].ranges[laserCenterStep - panStepTrimmed] < upperLimit)
              {
                obstructedLeft = true;
                obstructedLeftDist = laserScan[uavID].ranges[laserCenterStep - panStepTrimmed];
              }
              else if (laserScan[uavID].ranges[laserCenterStep - sideStep] < upperLimit)
              {
                obstructedLeft = true;
                obstructedLeftDist = laserScan[uavID].ranges[laserCenterStep - sideStep];
              }

              if (laserScan[uavID].ranges[laserCenterStep + panStepTrimmed] < upperLimit)
              {
                obstructedRight = true;
                obstructedRightDist = laserScan[uavID].ranges[laserCenterStep + panStepTrimmed];
              }
              else if (laserScan[uavID].ranges[laserCenterStep + sideStep] < upperLimit)
              {
                obstructedRight = true;
                obstructedRightDist = laserScan[uavID].ranges[laserCenterStep + sideStep];
              }

              // if the path is blocked, we need to devise a new partial target
              if (obstructedCenter || obstructedLeft || obstructedRight)
              {
                // if the quadrotor is in the same level as the target
                if (std::fabs(currentPose.z - partialTargetPose.z) <= maxPositionError
                    && (obstructedLeftDist < 6.0 || obstructedCenterDist < 6.0 || obstructedRightDist < 6.0))
                {
//                  // if the obstacle is at the (partial) target, then we deem the destination is unreachable
//                  double lowerLimit = (planarDistToTarget - safetyMargin) / std::cos(std::abs(panStepObstructed) * laserAngularResolution * M_PI / 180.0);
//                  if (obstructedDistance > lowerLimit)
//                  {
//                    mut_dest[uavID].lock();
//                    dest[uavID].x = currentPose.x;
//                    dest[uavID].y = currentPose.y;
//                    dest[uavID].z = currentPose.z;
//                    mut_dest[uavID].unlock();
//                    std::cout << "Destination unreachable [by the laser scanner]" << std::endl;
//                    mut_laser[uavID].unlock();
//                    break;
//                  }

                  if (!obstructedLeft)
                  {
//                    if (laserScan[uavID].ranges[330] > 4.0)
//                    {
//                      double angle = (540 - 330) * laserAngularResolution * M_PI / 180.0;
//                      double c = std::cos(angle);
//                      double s = std::sin(angle);
//                      double vxUnit = vx / planarDistToTarget;
//                      double vyUnit = vy / planarDistToTarget;
//
//                      partialTargetPose.x = currentPose.x + (vxUnit * c - vyUnit * s);
//                      partialTargetPose.y = currentPose.y + (vxUnit * s + vyUnit * c);
//
//                      std::cout << uavID << "L1) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//                      mut_laser[uavID].unlock();
//                      continue;
//                    }

                    if (laserScan[uavID].ranges[180] > 2.0)
                    {
                      mut_pose[uavID].lock();
                      vx = partialTargetPose.x - pose[uavID].x;
                      vy = partialTargetPose.y - pose[uavID].y;
                      planarDistToTarget = std::sqrt(vx * vx + vy * vy);
                      partialTargetPose.x = pose[uavID].x + (-vy / planarDistToTarget);
                      partialTargetPose.y = pose[uavID].y + (vx / planarDistToTarget);
                      partialTargetPose.z = pose[uavID].z + 0.5;
                      mut_pose[uavID].unlock();

//                      partialTargetPose.x = currentPose.x + (-vy / planarDistToTarget);
//                      partialTargetPose.y = currentPose.y + (vx / planarDistToTarget);

                      std::cout << uavID << "L2) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
                      mut_laser[uavID].unlock();
                      hasObstructedCenter = false;
                      continue;
                    }
                  }

                  if (!obstructedRight)
                  {
//                    if (laserScan[uavID].ranges[750] > 4.0)
//                    {
//                      double angle = (540 - 750) * laserAngularResolution * M_PI / 180.0;
//                      double c = std::cos(angle);
//                      double s = std::sin(angle);
//                      double vxUnit = vx / planarDistToTarget;
//                      double vyUnit = vy / planarDistToTarget;
//
//                      partialTargetPose.x = currentPose.x + (vxUnit * c - vyUnit * s);
//                      partialTargetPose.y = currentPose.y + (vxUnit * s + vyUnit * c);
//
//                      std::cout << uavID << "R1) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//                      mut_laser[uavID].unlock();
//                      continue;
//                    }

                    if (laserScan[uavID].ranges[900] > 2.0)
                    {
                      mut_pose[uavID].lock();
                      vx = partialTargetPose.x - pose[uavID].x;
                      vy = partialTargetPose.y - pose[uavID].y;
                      planarDistToTarget = std::sqrt(vx * vx + vy * vy);
                      partialTargetPose.x = pose[uavID].x + (vy / planarDistToTarget);
                      partialTargetPose.y = pose[uavID].y + (-vx / planarDistToTarget);
                      partialTargetPose.z = pose[uavID].z + 0.5;
                      mut_pose[uavID].unlock();

//                      partialTargetPose.x = currentPose.x + (vy / planarDistToTarget);
//                      partialTargetPose.y = currentPose.y + (-vx / planarDistToTarget);

                      std::cout << uavID << "R2) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
                      mut_laser[uavID].unlock();
                      hasObstructedCenter = false;
                      continue;
                    }
                  }
                }

                // if reach here, it means it hasn't got a new partial target
                if (obstructedCenterDist < 10.0)
                {
                  mut_pose[uavID].lock();
                  vx = partialTargetPose.x - pose[uavID].x;
                  vy = partialTargetPose.y - pose[uavID].y;
                  planarDistToTarget = std::sqrt(vx * vx + vy * vy);
                  partialTargetPose.x = pose[uavID].x + (-vx / planarDistToTarget) * (1.0 - obstructedCenterDist);
                  partialTargetPose.y = pose[uavID].y + (-vy / planarDistToTarget) * (1.0 - obstructedCenterDist);
                  partialTargetPose.z = pose[uavID].z + 0.5;
                  mut_pose[uavID].unlock();

//                  partialTargetPose.x = currentPose.x + (-vx / planarDistToTarget) * (1.0 - obstructedCenterDist);
//                  partialTargetPose.y = currentPose.y + (-vy / planarDistToTarget) * (1.0 - obstructedCenterDist);
//                  partialTargetPose.z = currentPose.z + 0.5;

                  std::cout << uavID << "CT) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
                  hasObstructedCenter = true;
                }
              }
            }
            mut_laser[uavID].unlock();
          }

          /* check the sonar sensors */
          float sonarRange;
          if (currentPose.z < partialTargetPose.z)  // going upward
          {
            mut_sonarUpward[uavID].lock();
            sonarRange = sonarUpwardRange[uavID];
            mut_sonarUpward[uavID].unlock();
          }
          else  // going downward
          {
            mut_sonarDownward[uavID].lock();
            sonarRange = sonarDownwardRange[uavID];
            mut_sonarDownward[uavID].unlock();
          }

          // if there is a vertical obstacle
          if (sonarRange < sonarMaxRange
              && std::abs(partialTargetPose.z - currentPose.z) > (sonarRange))
          {
            // if the quadrotor only needs to move vertically and is obstructed, then we deem the destination is unreachable
            if (planarDistToTarget <= maxPositionError)
            {
              mut_dest[uavID].lock();
              dest[uavID].x = currentPose.x;
              dest[uavID].y = currentPose.y;
              if (currentPose.z < partialTargetPose.z)  // going upward
              {
                dest[uavID].z = currentPose.z + (sonarRange - distFromObsVertical);
              }
              else  // going downward
              {
                dest[uavID].z = currentPose.z - (sonarRange - distFromObsVertical);
              }
              mut_dest[uavID].unlock();
              std::cout << "Destination unreachable [by the sonar sensor]" << std::endl;
              break;
            }

            // devise a new partial target
            partialTargetPose.x = (partialTargetPose.x + currentPose.x) / 2;
            partialTargetPose.y = (partialTargetPose.y + currentPose.y) / 2;
            if (currentPose.z < partialTargetPose.z)  // going upward
            {
              partialTargetPose.z = currentPose.z + (sonarRange - distFromObsVertical);
            }
            else  // going downward
            {
              partialTargetPose.z = currentPose.z - (sonarRange - distFromObsVertical);
            }
            std::cout << uavID << "S) " << partialTargetPose.x << ", " << partialTargetPose.y  << ", " << partialTargetPose.z << std::endl;
          }

          /* set the destination of the UAV */
          mut_dest[uavID].lock();
          dest[uavID].x = partialTargetPose.x;
          dest[uavID].y = partialTargetPose.y;
          dest[uavID].z = partialTargetPose.z;
          mut_dest[uavID].unlock();
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
