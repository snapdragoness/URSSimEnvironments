#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hector_uav_msgs/EnableMotors.h>

#include <hector_uav_msgs/LandingAction.h>
#include <hector_uav_msgs/PoseAction.h>
#include <hector_uav_msgs/TakeoffAction.h>

typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseClient;

int main(int argc, char **argv)
{
  // Initialize ROS and sets up a node
  ros::init(argc, argv, "fake_wearable");
  ros::NodeHandle nh;

  ros::ServiceClient motor_enable_service = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;
  std::cout << "Call EnableMotors service - " << motor_enable_service.call(srv) << std::endl;
  std::cout << "Success - " << (bool)srv.response.success << std::endl;

  boost::shared_ptr<TakeoffClient> takeoff_client_ = boost::shared_ptr<TakeoffClient>(new TakeoffClient(nh, "action/takeoff"));
  std::cout << "Wait for action/takeoff server - " << takeoff_client_->waitForServer(ros::Duration(5.0)) << std::endl;
  hector_uav_msgs::TakeoffGoal goal;
  takeoff_client_->sendGoal(goal);

  if (takeoff_client_->waitForResult(ros::Duration(5.0)))
  {
    ROS_INFO("Takeoff action finished");
  }
  else
  {
    ROS_INFO("Takeoff action did not finish before the time out");
  }

  boost::shared_ptr<PoseClient> pose_client = boost::shared_ptr<PoseClient>(new PoseClient(nh, "action/pose"));
  std::cout << "Wait for action/pose server - " << pose_client->waitForServer(ros::Duration(5.0)) << std::endl;
  hector_uav_msgs::PoseGoal pose_goal;
  pose_goal.target_pose.pose.position.z = 5.0;
  pose_client->sendGoal(pose_goal);

  if (pose_client->waitForResult(ros::Duration(5.0)))
  {
    ROS_INFO("Pose action finished");
  }
  else
  {
    ROS_INFO("Pose action did not finish before the time out");
  }

  return 0;
}
