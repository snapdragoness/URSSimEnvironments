#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_NAVIGATOR_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_NAVIGATOR_H_

#include "urs_wearable/pose.h"
#include "urs_wearable/controller.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

#include <boost/thread.hpp>

class Navigator {
  ros::NodeHandlePtr nh;
  std::string ns;         // namespace of the UAV

  ros::Subscriber laserSub;           // subscriber of the UAV's laser scanner
  ros::Subscriber sonarDownwardSub;   // subscriber of the UAV's downward sonar sensor
  ros::Subscriber sonarUpwardSub;     // subscriber of the UAV's upward sonar sensor

  /* for laser scan */
  sensor_msgs::LaserScan laserScan;             // range from 0.1 - 30 meters
  const double laserAngularResolution = 0.25;   // a degree for each step
  const int laserMeasurementSteps = 1080;       // total scan steps. This indicates the number of elements in laserScan.ranges
  const int laserCenterStep = laserMeasurementSteps / 2;  // center front of the laser scanner

  /*
    Legal indexes (that do not get blocked by the quadrotor's legs) of the laser scan are:
    23-335, 381-694, 737-1065

    which could be trimmed down to:
    30-330, 390-690, 750-1050

    Measurement Steps = 1080  ;   Total Degree = 270 degree
             540              ;             0 deg
              |               ;               |
      180 --------- 900       ;   -90 deg --------- 90 deg
            /   \             ;             /   \
           0    1080          ;       -135 deg   135 deg
  */

  /* for sonar scan */
  float sonarDownwardRange = 1.0;     // range from 0.03 - 3 meters
  float sonarUpwardRange = 1.0;       // range from 0.03 - 3 meters
  const float sonarMaxRange = 3.0;    // maximum value possible from sonar sensors

  /* for navigation */
  const double maxPositionError = 0.1;      // maximum position error in meter (only non-negative values)
  const double maxOrientationError = 1.0;   // maximum orientation error in degree (only non-negative values)
  const double distFromObs = 0.2;           // minimum distance to be kept from horizontal obstacles (only non-negative values)
  const double distFromObsVertical = 1.0;   // minimum distance to be kept from vertical obstacles (only non-negative values)
                                            // should be at least 1.0. Otherwise, it might not be able to brake in time before crashing
  const double quadrotorLength = 1.0;       // suppose the quadrotors have square shape
  const double safetyMargin = quadrotorLength / 2.0 + distFromObs;  // used in navigate() to make sure there is enough space
                                                                    // for the quadrotor at the target destination

  boost::thread navigationThread;

  // methods with an underscore in front are supposed to be threads
  void _readLaserScan(const sensor_msgs::LaserScanConstPtr& msg);
  void _readSonarDownward(const sensor_msgs::RangeConstPtr& msg);
  void _readSonarUpward(const sensor_msgs::RangeConstPtr& msg);
  void _navigate(Controller& controller, const Pose targetPose, const bool oriented);

  boost::mutex mut_laser;
  boost::mutex mut_sonarDownward;
  boost::mutex mut_sonarUpward;

public:
  Navigator(const std::string& ns);
  Navigator() : Navigator("") {}
  ~Navigator();

  // important methods
  void navigate(Controller& controller, const Pose targetPose, const bool oriented);
  void cancel();

  // auxiliary methods
  void setNamespace(const std::string& ns);

  // static methods
  static double getDistance(const Pose& from, const Pose& to);
  static double getYawDiff(double yaw1, double yaw2);
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_NAVIGATOR_H_ */
