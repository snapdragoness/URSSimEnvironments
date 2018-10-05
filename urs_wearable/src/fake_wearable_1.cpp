 // This program is used to substitute a wearable device for testing

#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <urs_wearable/LocationAdd.h>
#include <urs_wearable/location_table.h>
#include <urs_wearable/SetGoal.h>

const std::string SET_GOAL_SERVICE_NAME = "/urs_wearable/set_goal";

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "fake_wearable_1");
  ros::NodeHandle nh;

  LocationTable::location_id_type location_id_sw, location_id_ne;
  urs_wearable::PoseEuler pose_sw, pose_ne;
  pose_sw.position.x = -20;
  pose_sw.position.y = -20;
  pose_sw.position.z = 0;
  pose_ne.position.x = 20;
  pose_ne.position.y = 20;
  pose_ne.position.z = 20;

  ros::ServiceClient location_add_client = nh.serviceClient<urs_wearable::LocationAdd>("urs_wearable/add_location");
  urs_wearable::LocationAdd location_add_srv;

  location_add_srv.request.pose = pose_sw;
  if (location_add_client.call(location_add_srv))
  {
    location_id_sw = location_add_srv.response.location_id;
  }
  else
  {
    ROS_ERROR("Call /urs_wearable/location_add error");
  }

  location_add_srv.request.pose = pose_ne;
  if (location_add_client.call(location_add_srv))
  {
    location_id_ne = location_add_srv.response.location_id;
  }
  else
  {
    ROS_ERROR("Call /urs_wearable/location_add error");
  }

  LocationTable::location_id_type loc_id;
  urs_wearable::PoseEuler pose;
  pose.position.x = -10;
  pose.position.y = 5;
  pose.position.z = 2;
  location_add_srv.request.pose = pose;
  if (location_add_client.call(location_add_srv))
  {
    loc_id = location_add_srv.response.location_id;
  }
  else
  {
    ROS_ERROR("Call /urs_wearable/location_add error");
  }

  urs_wearable::SetGoal set_goal_srv;
  urs_wearable::Predicate pred;
  pred.type = urs_wearable::Predicate::TYPE_ACTIVE_REGION;
  pred.predicate_active_region.location_id_sw.value = location_id_sw;
  pred.predicate_active_region.location_id_ne.value = location_id_ne;
  pred.predicate_active_region.truth_value = true;
  set_goal_srv.request.goal.push_back(pred);

  pred.type = urs_wearable::Predicate::TYPE_TOOK_OFF;
  pred.predicate_took_off.drone_id.value = 0;
  pred.predicate_took_off.truth_value = true;
  set_goal_srv.request.goal.push_back(pred);

  pred.predicate_took_off.drone_id.value = 1;
  pred.predicate_took_off.truth_value = true;
  set_goal_srv.request.goal.push_back(pred);

  pred.type = urs_wearable::Predicate::TYPE_DRONE_AT;
  pred.predicate_drone_at.drone_id.value = 0;
  pred.predicate_drone_at.location_id.value = loc_id;
  pred.predicate_drone_at.truth_value = true;
  set_goal_srv.request.goal.push_back(pred);

  if (ros::service::call(SET_GOAL_SERVICE_NAME, set_goal_srv))
  {
    ROS_INFO("Call %s successfully", SET_GOAL_SERVICE_NAME.c_str());
  }
  else
  {
    ROS_INFO("Call %s failed", SET_GOAL_SERVICE_NAME.c_str());
  }

  return EXIT_SUCCESS;
}
