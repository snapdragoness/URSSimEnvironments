#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_COMMON_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_COMMON_H_

#include <array>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

std::string exec(const char* cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
  {
    throw std::runtime_error("popen() failed!");
  }
  while (!feof(pipe.get()))
  {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
      result += buffer.data();
  }
  return result;
}

void writeFile(const std::string& path_name, const std::string& content)
{
  std::ofstream file;
  file.open(path_name);
  file << content;
  file.close();
}

std::vector<std::string> tokenizeString(const std::string& str, const std::string& delimiters)
{
  std::vector<std::string> tokens;
  // Skip delimiters at beginning.
  std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
  // Find first "non-delimiter".
  std::string::size_type pos = str.find_first_of(delimiters, lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(lastPos, pos - lastPos));
    // Skip delimiters.  Note the "not_of"
    lastPos = str.find_first_not_of(delimiters, pos);
    // Find next "non-delimiter"
    pos = str.find_first_of(delimiters, lastPos);
  }
  return tokens;
}

std::string& ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  str.erase(0, str.find_first_not_of(chars));
  return str;
}

std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  str.erase(str.find_last_not_of(chars) + 1);
  return str;
}

std::string& trim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  return ltrim(rtrim(str, chars), chars);
}

void ros_info(const std::string& s)
{
  ROS_INFO("%s: %s", ros::this_node::getName().c_str(), s.c_str());
}

void ros_warn(const std::string& s)
{
  ROS_WARN("%s: %s", ros::this_node::getName().c_str(), s.c_str());
}

void ros_error(const std::string& s)
{
  ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), s.c_str());
}

template <typename T>
void retrieve(const std::string& key, T& value)
{
  if (!ros::param::get(key, value))
  {
    ros_error("Failed to retrieve " + key);
    std::exit(EXIT_FAILURE);
  }
}

double pointDistance2D(const geometry_msgs::Point& from, const geometry_msgs::Point& to)
{
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  return std::sqrt(dx * dx + dy * dy);
}

double pointDistance3D(const geometry_msgs::Point& from, const geometry_msgs::Point& to)
{
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double dz = to.z - from.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double quaternionToYaw(const geometry_msgs::Quaternion& orientation)
{
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

double yawDiff(double yaw1, double yaw2)
{
  double diff = std::fabs(yaw1 - yaw2);
  return (diff > M_PI)? M_PI + M_PI - diff: diff;
}

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_COMMON_H_ */

//////////////////////////////////////////////////////////////////////////////
/*
typedef struct Euler
{
  double roll;
  double pitch;
  double yaw;
} Euler;

Euler quaternionToEuler(const geometry_msgs::QuaternionConstPtr& q)
{
  Euler euler;

  double ysqr = q->y * q->y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (q->w * q->x + q->y * q->z);
  double t1 = +1.0 - 2.0 * (q->x * q->x + ysqr);
  euler.roll = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (q->w * q->y - q->z * q->x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  euler.pitch = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q->w * q->z + q->x * q->y);
  double t4 = +1.0 - 2.0 * (ysqr + q->z * q->z);
  euler.yaw = std::atan2(t3, t4);

  return euler;
}

double quaternionToYaw(const geometry_msgs::QuaternionConstPtr& q)
{
  double t3 = +2.0 * (q->w * q->z + q->x * q->y);
  double t4 = +1.0 - 2.0 * (q->y * q->y + q->z * q->z);

  //  yaw (z-axis rotation)
  //          PI/2
  //            ^
  //            |
  //  PI,-PI <--+--> 0
  //            |
  //            v
  //         -PI/2
  double yaw = std::atan2(t3, t4);

  //  return the non-negative version
  //          PI/2
  //            ^
  //            |
  //      PI <--+--> 0
  //            |
  //            v
  //          3PI/2
  return (yaw < 0.0)? yaw + 2 * M_PI: yaw;
}
*/
