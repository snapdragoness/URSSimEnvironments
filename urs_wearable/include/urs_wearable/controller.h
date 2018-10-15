#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_

#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Empty.h>
#include <urs_wearable/GetDest.h>
#include <urs_wearable/SetAltitude.h>
#include <urs_wearable/SetOrientation.h>
#include <urs_wearable/SetPosition.h>

class Controller {
  // read about adjusting PID coefficients at https://oscarliang.com/quadcopter-pid-explained-tuning
  const double P = 0.5;
  const double I = 0.0;
  const double D = 0.0;
  const double P_Q = 2.0;
  const double D_Q = 0.5;

  const double MAX_POSITION_ERROR = 0.2;      // maximum position error in meter (only non-negative values)
  const double MAX_ORIENTATION_ERROR = 1.0;   // maximum orientation error in degree (only non-negative values)

  geometry_msgs::Point position_prev_error_;
  geometry_msgs::Point position_integral_;
  geometry_msgs::Quaternion q_prev_error_;

  geometry_msgs::Pose dest_;
  std::mutex dest_mutex_;

  ros::Publisher cmd_pub_;

  ros::Subscriber pose_sub_;
  void pidControl(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Subscriber depth_image_sub_;
  std::atomic<bool> is_moving_{false};
  void readDepthImage(const sensor_msgs::Image::ConstPtr&);
  void avoidObstacle();
  void stop();

  ros::Subscriber sonar_height_sub_;
  ros::Subscriber sonar_upward_sub_;
  void readSonarHeight(const sensor_msgs::RangeConstPtr&);
  void readSonarUpward(const sensor_msgs::RangeConstPtr&);

  geometry_msgs::Pose getDest();
  void setAltitude(const double);
  void setOrientation(const double);
  void setPosition(const geometry_msgs::Point);
  void setPositionBare(const geometry_msgs::Point&);

  ros::ServiceServer get_dest_service_;
  ros::ServiceServer set_altitude_service_;
  ros::ServiceServer set_orientation_service_;
  ros::ServiceServer set_position_service_;
  ros::ServiceServer set_position_bare_service_;
  ros::ServiceServer stop_service_;
  bool getDestService(urs_wearable::GetDest::Request&, urs_wearable::GetDest::Response&);
  bool setAltitudeService(urs_wearable::SetAltitude::Request&, urs_wearable::SetAltitude::Response&);
  bool setOrientationService(urs_wearable::SetOrientation::Request&, urs_wearable::SetOrientation::Response&);
  bool setPositionService(urs_wearable::SetPosition::Request&, urs_wearable::SetPosition::Response&);
  bool setPositionBareService(urs_wearable::SetPosition::Request&, urs_wearable::SetPosition::Response&);
  bool stopService(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

public:
  Controller();
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_CONTROLLER_H_ */
