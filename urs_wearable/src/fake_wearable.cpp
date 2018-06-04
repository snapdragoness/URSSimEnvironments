// This program is to substitute a wearable device to test the execution monitor

#include "colormod.h"

#include "urs_wearable/SetDest.h"

#include <ros/ros.h>

#include <cstdlib>
#include <ctime>

#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "fake_wearable");
  ros::NodeHandle nh;

  urs_wearable::DestEuler dest;
  dest.pose.position.x = 0;
  dest.pose.position.y = 0;
  dest.pose.position.z = 5;
  dest.set_orientation = false;

  urs_wearable::SetDest setDest;
  setDest.request.uav_id.push_back(0);
  setDest.request.dest.push_back(dest);

  if (ros::service::call("/urs_wearable/set_dest", setDest))
  {
    std::cout << "Call /urs_wearable/set_dest successfully" << std::endl;
  }
  else
  {
    std::cout << "Call /urs_wearable/set_dest failed" << std::endl;
  }

  return 0;
}
