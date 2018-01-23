#include "urs_wearable/pose.h"
#include "urs_wearable/controller.h"
#include "urs_wearable/navigator.h"

#include <ros/ros.h>

#include <boost/tokenizer.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <termios.h>

void initTermios(int echo);
void resetTermios(void);
char getch_(int echo);
char getch(void);
char getche(void);

/* for keyboard teleop */
const unsigned int nUAV = 4;
unsigned int activeID = 0;    // the ID of the UAV to receive the control from keyboard
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

  Controller controller[nUAV];
  Navigator navigator[nUAV];

  for (unsigned int i = 0; i < nUAV; i++)
  {
    controller[i].setNamespace("/uav" + std::to_string(i));
    controller[i].start();
    navigator[i].setNamespace("/uav" + std::to_string(i));
  }

  std::cout << "Total number of UAVs: " << nUAV
            << " [ID: " << ((nUAV == 1)? "0": "0-" + std::to_string(nUAV - 1)) << "]" << std::endl;
  std::cout << "Active ID: " << activeID << std::endl;

  /* read and evaluate keyboard input */
  ros::Rate rate(10);
  while (ros::ok())
  {
    char key = getch();
    Pose destActive = controller[activeID].getDest();
    switch (key)
    {
      case 'w':
      case 'W':
        destActive.x += moveStep;
        controller[activeID].setDest(destActive, false);
        break;
      case 's':
      case 'S':
        destActive.x -= moveStep;
        controller[activeID].setDest(destActive, false);
        break;
      case 'a':
      case 'A':
        destActive.y += moveStep;
        controller[activeID].setDest(destActive, false);
        break;
      case 'd':
      case 'D':
        destActive.y -= moveStep;
        controller[activeID].setDest(destActive, false);
        break;
      case 'r':
      case 'R':
        destActive.z += moveStep;
        controller[activeID].setDest(destActive, false);
        break;
      case 'f':
      case 'F':
        destActive.z -= moveStep;
        controller[activeID].setDest(destActive, false);
        break;
      case 'q':
      case 'Q':
        destActive.yaw += rotateStep;
        controller[activeID].setDest(destActive, true);
        break;
      case 'e':
      case 'E':
        destActive.yaw -= rotateStep;
        controller[activeID].setDest(destActive, true);
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
            tokens.push_back((std::string)token);
          }

          // check the input
          try
          {
            if (tokens.size() == 2 && !tokens[0].compare("land"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                // TODO: should actually check from the downward sonar sensor and disable the motors
                // TODO: write takeoff function
                Pose dest = controller[uavID].getDest();
                dest.z = 0.5;
                controller[uavID].setDest(dest, false);
              }
            }
            else if (tokens.size() == 5 && !tokens[0].compare("move"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                Pose targetPose;
                Pose pose = controller[uavID].getPose();
                targetPose.x = pose.x + std::stod(tokens[2]);
                targetPose.y = pose.y + std::stod(tokens[3]);
                targetPose.z = pose.z + std::stod(tokens[4]);
                targetPose.yaw = pose.yaw;

                navigator[uavID].navigate(controller[uavID], targetPose, false);
              }
            }
            else if (tokens.size() == 5 && !tokens[0].compare("goto"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                Pose targetPose;
                Pose pose = controller[uavID].getPose();
                targetPose.x = std::stod(tokens[2]);
                targetPose.y = std::stod(tokens[3]);
                targetPose.z = std::stod(tokens[4]);
                targetPose.yaw = pose.yaw;

                navigator[uavID].navigate(controller[uavID], targetPose, false);
              }
            }
            else if (tokens.size() == 3 && !tokens[0].compare("rotate"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                double degree = std::stod(tokens[2]);
                Pose dest = controller[uavID].getDest();
                dest.yaw = degree * M_PI / 180.0;
                controller[uavID].setDest(dest, true);
              }
            }
            else if (tokens.size() == 2 && !tokens[0].compare("cancel"))
            {
              int uavID = std::stoi(tokens[1]);
              if (uavID >= 0 && uavID < nUAV)
              {
                navigator[uavID].cancel();;
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
