#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "urs_wearable/common.h"

#include <urs_wearable/AddArea.h>
#include <urs_wearable/AddLocation.h>
#include <urs_wearable/Feedback.h>
#include <urs_wearable/SetGoal.h>

const std::string SET_GOAL_SERVICE_NAME = "/urs_wearable/set_goal";

void feedbackCB(const urs_wearable::Feedback::ConstPtr& msg)
{
  ros_warn("status: " + std::to_string(msg->status));
}

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "fake_wearable");
  ros::NodeHandle nh;

  ros::Subscriber feedback_sub = nh.subscribe("/urs_wearable/feedback", 1, feedbackCB);
  ros::ServiceClient add_location_client = nh.serviceClient<urs_wearable::AddLocation>("/urs_wearable/add_location");
  ros::ServiceClient add_area_client = nh.serviceClient<urs_wearable::AddArea>("/urs_wearable/add_area");

  std::vector<std::uint8_t> loc_ids;
  urs_wearable::AddLocation add_location_srv;
  add_location_srv.request.pose.position.x = 10;
  add_location_srv.request.pose.position.y = 10;
  add_location_srv.request.pose.position.z = 5;
  if (add_location_client.call(add_location_srv))
  {
    loc_ids.push_back(add_location_srv.response.loc_id);
  }
  else
  {
    ros_error("Error in calling /urs_wearable/add_location");
  }

  add_location_srv.request.pose.position.x = 20;
  add_location_srv.request.pose.position.y = 20;
  if (add_location_client.call(add_location_srv))
  {
    loc_ids.push_back(add_location_srv.response.loc_id);
  }
  else
  {
    ros_error("Error in calling /urs_wearable/add_location");
  }

  std::vector<std::uint8_t> area_ids;
  urs_wearable::AddArea add_area_srv;
  add_area_srv.request.loc_id_left = loc_ids[0];
  add_area_srv.request.loc_id_right = loc_ids[1];
  if (add_area_client.call(add_area_srv))
  {
    area_ids.push_back(add_area_srv.response.area_id);
  }
  else
  {
    ros_error("Error in calling /urs_wearable/add_area");
  }

  urs_wearable::SetGoal set_goal_srv;
  urs_wearable::Predicate pred;
  pred.truth_value = true;

  pred.type = urs_wearable::Predicate::TYPE_SCANNED;
  pred.scanned.d.value = 0;
  pred.scanned.a.value = area_ids[0];
  set_goal_srv.request.goal.push_back(pred);

  pred.type = urs_wearable::Predicate::TYPE_HOVERED;
  pred.hovered.d.value = 1;
  set_goal_srv.request.goal.push_back(pred);

  if (!ros::service::call(SET_GOAL_SERVICE_NAME, set_goal_srv))
  {
    ros_error("Error in calling " + SET_GOAL_SERVICE_NAME);
  }

  ros::spin();

  return EXIT_SUCCESS;
}
