#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "urs_wearable/common.h"

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

  ros::ServiceClient location_add_client = nh.serviceClient<urs_wearable::AddLocation>("urs_wearable/add_location");
  urs_wearable::AddLocation add_location_srv;

  unsigned int loc_id;
  geometry_msgs::Pose pose;
  pose.position.x = -10;
  pose.position.y = 5;
  pose.position.z = 5;
  add_location_srv.request.poses.push_back(pose);
  if (location_add_client.call(add_location_srv))
  {
    loc_id = add_location_srv.response.location_id;
  }
  else
  {
    ros_error("error in calling /urs_wearable/location_add");
  }

  urs_wearable::SetGoal set_goal_srv;
  urs_wearable::Predicate pred;

  pred.type = urs_wearable::Predicate::TYPE_HOVERED;
  pred.hovered.d.value = 0;
  pred.hovered.truth_value = true;
  set_goal_srv.request.goal.push_back(pred);

  pred.hovered.d.value = 1;
  pred.hovered.truth_value = true;
  set_goal_srv.request.goal.push_back(pred);

  pred.type = urs_wearable::Predicate::TYPE_AT;
  pred.at.d.value = 0;
  pred.at.l.value = loc_id;
  pred.at.truth_value = true;
  set_goal_srv.request.goal.push_back(pred);

  if (!ros::service::call(SET_GOAL_SERVICE_NAME, set_goal_srv))
  {
    ros_error("error in calling " + SET_GOAL_SERVICE_NAME);
  }

  ros::spin();

  return EXIT_SUCCESS;
}
