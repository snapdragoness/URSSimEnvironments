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

#include <termios.h>

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

double quaternionToYaw(const geometry_msgs::Quaternion::ConstPtr& q);
void uavController(const geometry_msgs::PoseStamped::ConstPtr& msg, const int uavID);
void uavCommander(ros::Rate rate);

void initTermios(int echo);
void resetTermios(void);
char getch_(int echo);
char getch(void);
char getche(void);

const unsigned int nUAV = 4;    // the total number of the UAVs
std::string nsUAV[nUAV];        // namespaces of the UAVs

ros::Subscriber poseSub[nUAV];    // subscribers of each UAV's ground truth
geometry_msgs::Twist cmd[nUAV];   // a twist message to be sent to /cmd_vel

geometry_msgs::Pose pose[nUAV];   // UAV's position from ground truth
geometry_msgs::Pose dest[nUAV];   // manually-set destinations of the UAVs
bool destReached[nUAV] = {false}; // used in uavController to indicate that the UAV
                                  // has reached the manually-set destination

// the maximum error allowed to be considered reaching the destination
const double positionErrorTolerance = 0.5;
const double orientationErrorTolerance = 0.5;

/* for PID control */
PID pidConst = {0.5, 0.0002, 0.00005};
geometry_msgs::Pose error[nUAV];
geometry_msgs::Pose prevError[nUAV];
geometry_msgs::Pose proportional[nUAV];
geometry_msgs::Pose integral[nUAV];
geometry_msgs::Pose derivation[nUAV];

boost::mutex mut[nUAV];   // mutexs of each UAV

unsigned int activeID = 0;  // the ID of the UAV that receive gamepad-like control from keyboard
                            // possible values: 0 to nUAV-1
double moveStep = 0.5;
double rotateStep = 0.5;

int main(int argc, char **argv)
{
  // initializes ROS, and sets up a node
  ros::init(argc, argv, "keyboard_teleop");

  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

  if (nUAV == 0) {
    ROS_ERROR("No UAV to control");
    exit(EXIT_FAILURE);
  }

  /* Initialization */
  for (int i = 0; i < nUAV; i++)
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
      dest[i].position.x = msg->pose.position.x;
      dest[i].position.y = msg->pose.position.y;
      dest[i].position.z = msg->pose.position.z + 1.0;
      dest[i].orientation.z = quaternionToYaw(boost::make_shared<geometry_msgs::Quaternion>(msg->pose.orientation));

      error[i].position.x = 0;
      error[i].position.y = 0;
      error[i].position.z = 0;
      error[i].orientation.z = 0;

      destReached[i] = false;

      poseSub[i] = nh->subscribe<geometry_msgs::PoseStamped>(nsUAV[i] + "/ground_truth_to_tf/pose", 10,
                                                             boost::bind(uavController, _1, i));
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
  boost::thread commanderThread(uavCommander, 10);

  /* read and evaluate keyboard input */
  ros::Rate rate(10);
  while (ros::ok())
  {
    char key = getch();
    switch (key)
    {
      case 'w':
      case 'W':
        mut[activeID].lock();
        dest[activeID].position.x = pose[activeID].position.x + moveStep;
        destReached[activeID] = false;
        mut[activeID].unlock();
        break;
      case 's':
      case 'S':
        mut[activeID].lock();
        dest[activeID].position.x = pose[activeID].position.x - moveStep;
        destReached[activeID] = false;
        mut[activeID].unlock();
        break;
      case 'a':
      case 'A':
        mut[activeID].lock();
        dest[activeID].position.y = pose[activeID].position.y + moveStep;
        destReached[activeID] = false;
        mut[activeID].unlock();
        break;
      case 'd':
      case 'D':
        mut[activeID].lock();
        dest[activeID].position.y = pose[activeID].position.y - moveStep;
        destReached[activeID] = false;
        mut[activeID].unlock();
        break;
      case 'r':
      case 'R':
        mut[activeID].lock();
        dest[activeID].position.z = pose[activeID].position.z + moveStep;
        destReached[activeID] = false;
        mut[activeID].unlock();
        break;
      case 'f':
      case 'F':
        mut[activeID].lock();
        dest[activeID].position.z = pose[activeID].position.z - moveStep;
        destReached[activeID] = false;
        mut[activeID].unlock();
        break;
      case 'q':
      case 'Q':
        mut[activeID].lock();
        dest[activeID].orientation.z = pose[activeID].orientation.z + rotateStep;
        destReached[activeID] = false;
        mut[activeID].unlock();
        break;
      case 'e':
      case 'E':
        mut[activeID].lock();
        dest[activeID].orientation.z = pose[activeID].orientation.z - rotateStep;
        destReached[activeID] = false;
        mut[activeID].unlock();
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
                mut[uavID].lock();
                dest[uavID].position.z = 0.5;
                destReached[uavID] = false;
                mut[uavID].unlock();
              }
            }
            else if (tokens.size() == 5 && !tokens[0].compare("move"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                mut[uavID].lock();
                dest[uavID].position.x += std::stod(tokens[2]);
                dest[uavID].position.y += std::stod(tokens[3]);
                dest[uavID].position.z += std::stod(tokens[4]);
                destReached[uavID] = false;
                mut[uavID].unlock();
              }
            }
            else if (tokens.size() == 5 && !tokens[0].compare("goto"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                mut[uavID].lock();
                dest[uavID].position.x = std::stod(tokens[2]);
                dest[uavID].position.y = std::stod(tokens[3]);
                dest[uavID].position.z = std::stod(tokens[4]);
                destReached[uavID] = false;
                mut[uavID].unlock();
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
}

/*
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

double quaternionToYaw(const geometry_msgs::Quaternion::ConstPtr& q)
{
  double ysqr = q->y * q->y;

  double t3 = +2.0 * (q->w * q->z + q->x * q->y);
  double t4 = +1.0 - 2.0 * (ysqr + q->z * q->z);

  // yaw (z-axis rotation)
  //           PI/2
  //             ^
  //             |
  //  PI, -PI <--+--> 0
  //             |
  //             v
  //          -PI/2
  double yaw = std::atan2(t3, t4);
  return (yaw < 0.0)? yaw + 2 * M_PI: yaw;
}

// PID Control
void uavController(const geometry_msgs::PoseStamped::ConstPtr& msg, int uavID)
{
  double yaw = quaternionToYaw(boost::make_shared<geometry_msgs::Quaternion>(msg->pose.orientation));

  mut[uavID].lock();    // lock pose
  pose[uavID].position.x = msg->pose.position.x;
  pose[uavID].position.y = msg->pose.position.y;
  pose[uavID].position.z = msg->pose.position.z;
  pose[uavID].orientation.z = yaw;  // this is NOT the z component of a quaternion
  mut[uavID].unlock();  // unlock pose

  prevError[uavID].position.x = error[uavID].position.x;
  prevError[uavID].position.y = error[uavID].position.y;
  prevError[uavID].position.z = error[uavID].position.z;
  prevError[uavID].orientation.z = error[uavID].orientation.z;

  mut[uavID].lock();    // lock dest
  error[uavID].position.x = dest[uavID].position.x - pose[uavID].position.x;
  error[uavID].position.y = dest[uavID].position.y - pose[uavID].position.y;
  error[uavID].position.z = dest[uavID].position.z - pose[uavID].position.z;
  error[uavID].orientation.z = dest[uavID].orientation.z - pose[uavID].orientation.z;
  mut[uavID].unlock();  // unlock dest

  if (error[uavID].orientation.z < -1.0 * M_PI) {
    error[uavID].orientation.z += M_PI + M_PI;
  } else if (error[uavID].orientation.z > M_PI) {
    error[uavID].orientation.z -= M_PI + M_PI;
  }

  proportional[uavID].position.x = pidConst.p * error[uavID].position.x;
  proportional[uavID].position.y = pidConst.p * error[uavID].position.y;
  proportional[uavID].position.z = pidConst.p * error[uavID].position.z;
  proportional[uavID].orientation.z = pidConst.p * error[uavID].orientation.z;
  integral[uavID].position.x += pidConst.i * error[uavID].position.x;
  integral[uavID].position.y += pidConst.i * error[uavID].position.y;
  integral[uavID].position.z += pidConst.i * error[uavID].position.z;
  integral[uavID].orientation.z += pidConst.i * error[uavID].orientation.z;
  derivation[uavID].position.x = pidConst.d * (error[uavID].position.x - prevError[uavID].position.x);
  derivation[uavID].position.y = pidConst.d * (error[uavID].position.y - prevError[uavID].position.y);
  derivation[uavID].position.z = pidConst.d * (error[uavID].position.z - prevError[uavID].position.z);
  derivation[uavID].orientation.z = pidConst.d * (error[uavID].orientation.z - prevError[uavID].orientation.z);

  double x, y, z, oz;
  x = proportional[uavID].position.x + integral[uavID].position.x + derivation[uavID].position.x;
  y = proportional[uavID].position.y + integral[uavID].position.y + derivation[uavID].position.y;
  z = proportional[uavID].position.z + integral[uavID].position.z + derivation[uavID].position.z;
  oz = proportional[uavID].orientation.z + integral[uavID].orientation.z + derivation[uavID].orientation.z;

  double rx, ry;
  rx = x * std::cos(-1 * yaw) - y * std::sin(-1 * yaw);
  ry = x * std::sin(-1 * yaw) + y * std::cos(-1 * yaw);

  mut[uavID].lock();    // lock cmd, destReached
  cmd[uavID].linear.x = rx;
  cmd[uavID].linear.y = ry;
  cmd[uavID].linear.z = z;
  cmd[uavID].angular.z = oz * 4;

  if (!destReached[uavID] &&
      std::fabs(error[uavID].position.x) < positionErrorTolerance &&
      std::fabs(error[uavID].position.y) < positionErrorTolerance &&
      std::fabs(error[uavID].position.z) < positionErrorTolerance &&
      std::fabs(error[uavID].orientation.z) < orientationErrorTolerance) {
    destReached[uavID] = true;
  }
  mut[uavID].unlock();  // unlock cmd, destReached
}

void uavCommander(ros::Rate rate)
{
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::Publisher cmdPub[nUAV];

  for (int i = 0; i < nUAV; i++)
  {
    cmdPub[i] = nh->advertise<geometry_msgs::Twist>(nsUAV[i] + "/cmd_vel", 100, false);
  }

  while (ros::ok())
  {
    for (int i = 0; i < nUAV; i++)
    {
      mut[i].lock();    // lock cmd
      cmdPub[i].publish(cmd[i]);
      mut[i].unlock();  // unlock cmd
    }
    ros::spinOnce();
    rate.sleep();
  }
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

