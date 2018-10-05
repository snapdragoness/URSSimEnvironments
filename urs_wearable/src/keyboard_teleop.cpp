#include <boost/tokenizer.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <termios.h>

#include <ros/ros.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <urs_wearable/GetDest.h>
#include <urs_wearable/PoseEuler.h>
#include <urs_wearable/SetAltitude.h>
#include <urs_wearable/SetDest.h>

#include "urs_wearable/common.h"

struct termios old_t, new_t;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
  tcgetattr(0, &old_t);                 // grab old terminal i/o settings
  new_t = old_t;                        // make new settings same as old settings
  new_t.c_lflag &= ~ICANON;             // disable buffered i/o
  new_t.c_lflag &= echo ? ECHO : ~ECHO; // set echo mode
  tcsetattr(0, TCSANOW, &new_t);        // use these new terminal i/o settings now
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

int main(int argc, char **argv)
{
  /* initialize ROS, and sets up a node */
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int n_uav;
  private_nh.param<int>("n_uav", n_uav, 4);

  std::string ns;
  private_nh.param<std::string>("ns", ns, "/uav");

  for (unsigned int i = 0; i < n_uav; i++)
  {
    // Enable motors
    hector_uav_msgs::EnableMotors enable_motors_srv;
    enable_motors_srv.request.enable = true;
    if (!ros::service::call(ns + std::to_string(i) + "/enable_motors", enable_motors_srv) ||
        !enable_motors_srv.response.success)
    {
      ros_error("Error in calling service " + ns + std::to_string(i) + "/enable_motors");
      return EXIT_FAILURE;
    }

    // Set altitude
    urs_wearable::SetAltitude set_altitude_srv;
    set_altitude_srv.request.z = 4.0;
    if (!ros::service::call(ns + std::to_string(i) + "/set_altitude", set_altitude_srv))
    {
      ros_error("Error in calling service " + ns + std::to_string(i) + "/set_altitude");
      return EXIT_FAILURE;
    }
  }

  unsigned int active_id = 0;
  double move_step = 0.5;
  double rotate_step = 0.5;

  ros_info("Total number of drones: " + std::to_string(n_uav) +
           " [ID: " + ((n_uav == 1) ? "0" : "0 - " + std::to_string(n_uav - 1)) + "]");
  ros_info("Active ID: " + std::to_string(active_id));

  /* read and evaluate keyboard input */
  ros::Rate rate(10);
  while (ros::ok())
  {
    char key = getch();

    urs_wearable::GetDest get_dest_srv;
    if (!ros::service::call(ns + std::to_string(active_id) + "/get_dest", get_dest_srv))
    {
      ros_error("Error in calling service " + ns + std::to_string(active_id) + "/get_dest");
      return EXIT_FAILURE;
    }
    urs_wearable::SetDest set_dest_srv;
    set_dest_srv.request.dest = get_dest_srv.response.dest;

    switch (key)
    {
      case 'w':
      case 'W':
        set_dest_srv.request.dest.position.x += move_step;
        ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
        break;
      case 's':
      case 'S':
        set_dest_srv.request.dest.position.x -= move_step;
        ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
        break;
      case 'a':
      case 'A':
        set_dest_srv.request.dest.position.y += move_step;
        ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
        break;
      case 'd':
      case 'D':
        set_dest_srv.request.dest.position.y -= move_step;
        ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
        break;
      case 'r':
      case 'R':
        set_dest_srv.request.dest.position.z += move_step;
        ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
        break;
      case 'f':
      case 'F':
        set_dest_srv.request.dest.position.z -= move_step;
        ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
        break;
      case 'q':
      case 'Q':
        set_dest_srv.request.dest.orientation.z += rotate_step;
        ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
        // TODO: Add set_orientation service
        break;
      case 'e': case 'E':
        set_dest_srv.request.dest.orientation.z -= rotate_step;
        ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
        // TODO: Add set_orientation service
        break;
      case 't': case 'T':
        move_step += 0.1;
        ros_info("Move step = " + std::to_string(move_step));
        break;
      case 'g': case 'G':
        move_step -= 0.1;
        ros_info("Move step = " + std::to_string(move_step));
        break;
      case 'y': case 'Y':
        rotate_step += 0.1;
        ros_info("Rotate step = " + std::to_string(rotate_step));
        break;
      case 'h': case 'H':
        rotate_step -= 0.1;
        ros_info("Rotate step = " + std::to_string(rotate_step));
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
        ros_warn("keyboard input = " + keyboardInput);

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
            if (tokens.size() == 5 && !tokens[0].compare("goto"))
            {
              int id = std::stoi(tokens[1]);
              if (id >= 0 && id < n_uav)
              {
                set_dest_srv.request.dest.position.x = std::stod(tokens[2]);
                set_dest_srv.request.dest.position.y = std::stod(tokens[3]);
                set_dest_srv.request.dest.position.z = std::stod(tokens[4]);

                ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
              }
            }
            else if (tokens.size() == 2 && !tokens[0].compare("land"))
            {
              int id = std::stoi(tokens[1]);
              if (id >= 0 && id < n_uav)
              {
                // TODO: Add landing service
              }
            }
            else if (tokens.size() == 3 && !tokens[0].compare("rotate"))
            {
              int id = std::stoi(tokens[1]);
              if (id >= 0 && id < n_uav)
              {
                set_dest_srv.request.dest.orientation.z = std::stod(tokens[2]) * M_PI / 180.0;
                ros::service::call(ns + std::to_string(active_id) + "/set_dest", set_dest_srv);
              }
            }
            else if (tokens.size() == 2 && !tokens[0].compare("stop"))
            {
              int id = std::stoi(tokens[1]);
              if (id >= 0 && id < n_uav)
              {
                // TODO: Add stop service
              }
            }
            else if (tokens.size() == 2 && !tokens[0].compare("takeoff"))
            {
              int id = std::stoi(tokens[1]);
              if (id >= 0 && id < n_uav)
              {
                // TODO: Add takeoff service
              }
            }
            else if (tokens.size() == 1)
            {
              int id = std::stoi(tokens[0]);
              if (id >= 0 && id < n_uav)
              {
                active_id = id;
                ros_info("Active ID: " + std::to_string(active_id));
              }
            }
          }
          catch (...)
          {
            ros_error("Error in parsing the input");
          }
        }
        break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}
