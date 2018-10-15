#include <ros/ros.h>
#include <urs_wearable/AddLocation.h>
#include <urs_wearable/SetGoal.h>

const std::string SET_GOAL_SERVICE_NAME = "/urs_wearable/set_goal";

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "fake_wearable_2");
  ros::NodeHandle nh;

  urs_wearable::SetGoal set_goal_srv;

  urs_wearable::Predicate pred;
  pred.type = urs_wearable::Predicate::TYPE_TOOK_OFF;
  pred.predicate_took_off.truth_value = true;
  pred.predicate_took_off.drone_id.value = 0;
  set_goal_srv.request.goal.push_back(pred);

  pred.predicate_took_off.drone_id.value = 1;
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
