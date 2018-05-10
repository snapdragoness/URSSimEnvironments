#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_POSE_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_POSE_H_

typedef struct Pose
{
  double x, y, z;
  double yaw;
  double pitch;
} Pose;

typedef struct Waypoint
{
  Pose pose;
  bool rotate;
} Waypoint;

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_POSE_H_ */
