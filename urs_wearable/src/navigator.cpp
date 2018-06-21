#include "urs_wearable/navigator.h"

#include <atomic>
#include <cmath>
#include <iostream>

#include <chrono>
#include <thread>

Navigator::Navigator(const std::string& ns)
{
  this->ns = ns;
}

Navigator::~Navigator()
{
  cancel();
}

void Navigator::navigate(Controller& controller, const urs_wearable::PoseEuler targetPose, const bool oriented)
{
  cancel();  // cancel the old navigation thread, if any

  depthImageSub = nh->subscribe(ns + "/camera/depth/image_raw", 10, &Navigator::_readDepthImage, this);
//  laserSub = nh->subscribe<sensor_msgs::LaserScan>(ns + "/scan", 10, &Navigator::_readLaserScan, this);
  sonarDownwardSub = nh->subscribe<sensor_msgs::Range>(ns + "/sonar_height", 10, &Navigator::_readSonarDownward, this);
  sonarUpwardSub = nh->subscribe<sensor_msgs::Range>(ns + "/sonar_upward", 10, &Navigator::_readSonarUpward, this);
  navigationThread = boost::thread(&Navigator::_navigate, this, boost::ref(controller), targetPose, oriented);
}

std::atomic<bool> hasObstacle(false);
void Navigator::_readDepthImage(const sensor_msgs::Image::ConstPtr& msg)
{
  if (!msg->encoding.compare("32FC1"))
  {
    /* https://answers.ros.org/question/246066/how-can-i-get-object-distance-using-cameradepthimage_raw
     * depth_array[0], depth_array[1], ..., depth_aray[image.height * image.width - 1] */
//    mut_depthImage.lock();
    depthImageArray = reinterpret_cast<const float *>(&(msg->data[0]));

    bool obstructed = false;
//    for (int i = 0; i < 120; i++)
//    {
//      float depth = depthImageArray[640 * (i + 0) + 320];
//      if (!std::isnan(depth) && depth < 9.0)
//      {
//        obstructed = true;
//        std::cout << "obstructed at row = " << (i + 0) << ", depth = " << depth << std::endl;
//        break;
//      }
//    }
    if (!std::isnan(depthImageArray[320]))
    {
      obstructed = true;
    }
    hasObstacle = obstructed;

//    mut_depthImage.unlock();

    // TODO: check obstacle
  }
  else
  {
    ROS_ERROR("Image data encoding not supported");
  }
}

void Navigator::_navigate(Controller& controller, const urs_wearable::PoseEuler targetPose, const bool oriented)
{
  std::cout << "A navigation thread " << ns << " to ["
      << targetPose.position.x << ", " << targetPose.position.y << ", " << targetPose.position.z << "] has started" << std::endl;

  try
  {
    double vx = targetPose.position.x - controller.getPose().position.x;
    double vy = targetPose.position.y - controller.getPose().position.y;
    double yawHeaded = std::atan2(vy, vx);
    if (yawHeaded < 0.0)
    {
      yawHeaded += M_PI + M_PI;
    }

    double planarDistToTarget = std::sqrt(vx * vx + vy * vy);
    if (planarDistToTarget > maxPositionError)
    {
      urs_wearable::PoseEuler dest = controller.getDest();
      dest.orientation.z = yawHeaded;
      controller.setDest(dest, true);

      do
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } while (Navigator::getYawDiff(controller.getPose().orientation.z, yawHeaded) > maxOrientationError * M_PI / 180.0);
    }

    while (true)
    {
      vx = targetPose.position.x - controller.getPose().position.x;
      vy = targetPose.position.y - controller.getPose().position.y;
      planarDistToTarget = std::sqrt(vx * vx + vy * vy);

      // move horizontally
      if (planarDistToTarget > maxPositionError)
      {
        yawHeaded = std::atan2(vy, vx);
        if (yawHeaded < 0.0)
        {
          yawHeaded += M_PI + M_PI;
        }

        urs_wearable::PoseEuler partialTargetPose = targetPose;
        partialTargetPose.position.z = controller.getPose().position.z;
        partialTargetPose.orientation.z = yawHeaded;

        controller.setDest(partialTargetPose, true);
      }
      else  // move vertically
      {
        controller.setDest(targetPose, true);
        while (Navigator::getDistance(controller.getPose(), targetPose) > maxPositionError)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

      if (Navigator::getDistance(controller.getPose(), targetPose) <= maxPositionError)
      {
        if (oriented)
        {
          while (Navigator::getYawDiff(controller.getPose().orientation.z, targetPose.orientation.z) > maxOrientationError * M_PI / 180.0)
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
        break;
      }

      if (hasObstacle)
      {
        urs_wearable::PoseEuler currentPose = controller.getPose();
        vx = targetPose.position.x - currentPose.position.x;
        vy = targetPose.position.y - currentPose.position.y;
        planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//        currentPose.x = currentPose.x - (vx / planarDistToTarget) * 2.0;
//        currentPose.y = currentPose.y - (vy / planarDistToTarget) * 2.0;
//        currentPose.z++;
        controller.setDest(currentPose, false);

        while (Navigator::getDistance(controller.getPose(), currentPose) > maxPositionError)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

//        currentPose.z += 100.0; // fly up to avoid obstacle
//        controller.setDest(currentPose, false);

        while (hasObstacle)
        {
          currentPose.position.z++;
          controller.setDest(currentPose, false);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

//        controller.setDest(controller.getPose(), false);
//        while (Navigator::getDistance(controller.getPose(), currentPose) > maxPositionError)
//        {
//          boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
//        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  catch (boost::thread_interrupted&)
  {
    std::cout << "A navigation thread " << ns << " to ["
        << targetPose.position.x << ", " << targetPose.position.y << ", " << targetPose.position.z << "] has been interrupted" << std::endl;
  }
  std::cout << "A navigation thread " << ns << " to ["
      << targetPose.position.x << ", " << targetPose.position.y << ", " << targetPose.position.z << "] has terminated" << std::endl;
}

//void Navigator::_navigate(Controller& controller, const urs_wearable::Pose targetPose, const bool oriented)
//{
//  std::cout << "A navigation thread " << ns << " to ["
//      << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has started" << std::endl;
//
//  try
//  {
//    urs_wearable::Pose partialTargetPose;
//    partialTargetPose.x = targetPose.x;
//    partialTargetPose.y = targetPose.y;
//    partialTargetPose.z = targetPose.z;
//
//    bool hasObstructedCenter = false;
//
//    // (int)(std::atan2(0.7, 2.0) * 180.0 / M_PI / 0.25) + 1 = 78
//    // (int)(std::atan2(0.5, 1.0) * 180.0 / M_PI / 0.25) + 1 = 57
//    // (int)(std::atan2(0.7, 1.0) * 180.0 / M_PI / 0.25) + 1 = 140
//    int sideStep = (int)(std::atan2(safetyMargin, 2.0) * 180.0 / M_PI / depthImageAngularResolution) + 1;
//
//    while (true)
//    {
//      urs_wearable::Pose currentPose = controller.getPose();
//
//      // if the UAV reaches the target destination
//      if (Navigator::getDistance(currentPose, targetPose) <= maxPositionError)
//      {
//        if (!oriented || Navigator::getYawDiff(currentPose.yaw, targetPose.yaw) <= maxOrientationError * M_PI / 180.0) {
//          break;
//        }
//        else
//        {
//          urs_wearable::Pose dest = controller.getDest();
//          dest.yaw = targetPose.yaw;
//          controller.setDest(dest, true);
//        }
//      }
//      else  // otherwise, begin navigating
//      {
//        // update the partial target destination if the UAV has reached it
//        if (Navigator::getDistance(currentPose, partialTargetPose) <= maxPositionError)
//        {
//          if (hasObstructedCenter)
//          {
//            partialTargetPose.x = (targetPose.x + currentPose.x) / 2;
//            partialTargetPose.y = (targetPose.y + currentPose.y) / 2;
//            partialTargetPose.z = currentPose.z + 0.5;
//            hasObstructedCenter = false;
//          }
//          else
//          {
//            partialTargetPose.x = targetPose.x;
//            partialTargetPose.y = targetPose.y;
//            partialTargetPose.z = targetPose.z;
//          }
////            std::cout << ns << "RE) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//        }
//
//        double vx = partialTargetPose.x - currentPose.x;
//        double vy = partialTargetPose.y - currentPose.y;
//        double planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//        double yawHeaded = std::atan2(vy, vx);
//        if (yawHeaded < 0.0)
//        {
//          yawHeaded += M_PI + M_PI;
//        }
//
//        // rotate only if it needs to move horizontally
//        if (planarDistToTarget > maxPositionError)
//        {
//          urs_wearable::Pose dest = controller.getDest();
//          dest.yaw = yawHeaded;
//          controller.setDest(dest, true);
//        }
//
//        // if the quadrotor has aligned itself to the partial target destination
//        if (Navigator::getYawDiff(currentPose.yaw, yawHeaded) <= M_PI / 180.0  // allow 1.0 degree of error
//            || planarDistToTarget <= maxPositionError)              // or only needs to fly vertically
//        {
//          if (planarDistToTarget > maxPositionError)  // if needs to move horizontally
//          {
//            double panDegree = std::atan2(safetyMargin, planarDistToTarget) * 180.0 / M_PI;
//            int panStep = (int)(panDegree / depthImageAngularResolution) + 1;
//
//            // legal range: 390 <- 540 -> 690. The maximum steps from 540 to both side are 150
//            int panStepTrimmed = (panStep >= 320)? 319: panStep;
//
//            // TODO: In the case that panSteps > 150,
//            // there might be a problem if the quadrotor is asked to go through a passage than it can't squeeze through.
//
//            /* check the laser scanner */
//            mut_depthImage.lock();
//            if (!depthImageArray)
//            {
//              bool obstructedCenter = false;
//              bool obstructedLeft = false;
//              bool obstructedRight = false;
//              double obstructedCenterDist = DBL_MAX;
//              double obstructedLeftDist = DBL_MAX;
//              double obstructedRightDist = DBL_MAX;
//
//              // TODO
//              int vcenter = 35;
//
//              if (!std::isnan(depthImageArray[640*vcenter + depthImageCenterStep])
//                && depthImageArray[640*vcenter + depthImageCenterStep] < planarDistToTarget + safetyMargin)
//              {
//                obstructedCenter = true;
//                obstructedCenterDist = depthImageArray[640*vcenter + depthImageCenterStep];
//              }
//
//              double upperLimit = (planarDistToTarget + safetyMargin) / std::cos(panStepTrimmed * depthImageAngularResolution * M_PI / 180.0);
//              if (!std::isnan(depthImageArray[640*vcenter + depthImageCenterStep] - panStepTrimmed)
//                && depthImageArray[640*vcenter + depthImageCenterStep - panStepTrimmed] < upperLimit)
//              {
//                obstructedLeft = true;
//                obstructedLeftDist = depthImageArray[640*vcenter + depthImageCenterStep - panStepTrimmed];
//              }
//              else if (!std::isnan(depthImageArray[640*vcenter + depthImageCenterStep - sideStep])
//                && depthImageArray[640*vcenter + depthImageCenterStep - sideStep] < upperLimit)
//              {
//                obstructedLeft = true;
//                obstructedLeftDist = depthImageArray[640*vcenter + depthImageCenterStep - sideStep];
//              }
//
//              if (!std::isnan(depthImageArray[640*vcenter + depthImageCenterStep + panStepTrimmed])
//                && depthImageArray[640*vcenter + depthImageCenterStep + panStepTrimmed] < upperLimit)
//              {
//                obstructedRight = true;
//                obstructedRightDist = depthImageArray[640*vcenter + depthImageCenterStep + panStepTrimmed];
//              }
//              else if (!std::isnan(depthImageArray[640*vcenter + depthImageCenterStep + sideStep])
//                && depthImageArray[640*vcenter + depthImageCenterStep + sideStep] < upperLimit)
//              {
//                obstructedRight = true;
//                obstructedRightDist = depthImageArray[640*vcenter + depthImageCenterStep + sideStep];
//              }
//
//              // if the path is blocked, we need to devise a new partial target
//              if (obstructedCenter || obstructedLeft || obstructedRight)
//              {
//                // if the quadrotor is in the same level as the target
//                if (std::fabs(currentPose.z - partialTargetPose.z) <= maxPositionError
//                    && (obstructedLeftDist < 6.0 || obstructedCenterDist < 6.0 || obstructedRightDist < 6.0))
//                {
////                  // if the obstacle is at the (partial) target, then we deem the destination is unreachable
////                  double lowerLimit = (planarDistToTarget - safetyMargin) / std::cos(std::abs(panStepObstructed) * laserAngularResolution * M_PI / 180.0);
////                  if (obstructedDistance > lowerLimit)
////                  {
////                    mut_dest[uavID].lock();
////                    dest[uavID].x = currentPose.x;
////                    dest[uavID].y = currentPose.y;
////                    dest[uavID].z = currentPose.z;
////                    mut_dest[uavID].unlock();
////                    std::cout << "Destination unreachable [by the laser scanner]" << std::endl;
////                    mut_laser[uavID].unlock();
////                    break;
////                  }
//
//                  if (!obstructedLeft)
//                  {
////                    if (laserScan[uavID].ranges[330] > 4.0)
////                    {
////                      double angle = (540 - 330) * laserAngularResolution * M_PI / 180.0;
////                      double c = std::cos(angle);
////                      double s = std::sin(angle);
////                      double vxUnit = vx / planarDistToTarget;
////                      double vyUnit = vy / planarDistToTarget;
////
////                      partialTargetPose.x = currentPose.x + (vxUnit * c - vyUnit * s);
////                      partialTargetPose.y = currentPose.y + (vxUnit * s + vyUnit * c);
////
////                      std::cout << ns << "L1) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
////                      mut_laser[uavID].unlock();
////                      continue;
////                    }
//
//                    if (!std::isnan(depthImageArray[640*vcenter])
//                      && depthImageArray[640*vcenter] > 2.0)
//                    {
//                      urs_wearable::Pose pose = controller.getPose();
//                      vx = partialTargetPose.x - pose.x;
//                      vy = partialTargetPose.y - pose.y;
//                      planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//                      partialTargetPose.x = pose.x + (-vy / planarDistToTarget);
//                      partialTargetPose.y = pose.y + (vx / planarDistToTarget);
//                      partialTargetPose.z = pose.z + 0.5;
//
////                      partialTargetPose.x = currentPose.x + (-vy / planarDistToTarget);
////                      partialTargetPose.y = currentPose.y + (vx / planarDistToTarget);
//
////                      std::cout << ns << "L2) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//                      mut_depthImage.unlock();
//                      hasObstructedCenter = false;
//                      continue;
//                    }
//                  }
//
//                  if (!obstructedRight)
//                  {
////                    if (laserScan[uavID].ranges[750] > 4.0)
////                    {
////                      double angle = (540 - 750) * laserAngularResolution * M_PI / 180.0;
////                      double c = std::cos(angle);
////                      double s = std::sin(angle);
////                      double vxUnit = vx / planarDistToTarget;
////                      double vyUnit = vy / planarDistToTarget;
////
////                      partialTargetPose.x = currentPose.x + (vxUnit * c - vyUnit * s);
////                      partialTargetPose.y = currentPose.y + (vxUnit * s + vyUnit * c);
////
////                      std::cout << uavID << "R1) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
////                      mut_laser[uavID].unlock();
////                      continue;
////                    }
//
//                    if (!std::isnan(depthImageArray[640*vcenter + 639])
//                      && depthImageArray[640*vcenter + 639] > 2.0)
//                    {
//                      urs_wearable::Pose pose = controller.getPose();
//                      vx = partialTargetPose.x - pose.x;
//                      vy = partialTargetPose.y - pose.y;
//                      planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//                      partialTargetPose.x = pose.x + (vy / planarDistToTarget);
//                      partialTargetPose.y = pose.y + (-vx / planarDistToTarget);
//                      partialTargetPose.z = pose.z + 0.5;
//
////                      partialTargetPose.x = currentPose.x + (vy / planarDistToTarget);
////                      partialTargetPose.y = currentPose.y + (-vx / planarDistToTarget);
//
////                      std::cout << ns << "R2) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//                      mut_depthImage.unlock();
//                      hasObstructedCenter = false;
//                      continue;
//                    }
//                  }
//                }
//
//                // if reach here, it means it hasn't got a new partial target
//                if (obstructedCenterDist < 10.0)
//                {
//                  urs_wearable::Pose pose = controller.getPose();
//                  vx = partialTargetPose.x - pose.x;
//                  vy = partialTargetPose.y - pose.y;
//                  planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//                  partialTargetPose.x = pose.x + (-vx / planarDistToTarget) * (1.0 - obstructedCenterDist);
//                  partialTargetPose.y = pose.y + (-vy / planarDistToTarget) * (1.0 - obstructedCenterDist);
//                  partialTargetPose.z = pose.z + 0.5;
//
////                  partialTargetPose.x = currentPose.x + (-vx / planarDistToTarget) * (1.0 - obstructedCenterDist);
////                  partialTargetPose.y = currentPose.y + (-vy / planarDistToTarget) * (1.0 - obstructedCenterDist);
////                  partialTargetPose.z = currentPose.z + 0.5;
//
////                  std::cout << ns << "CT) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//                  hasObstructedCenter = true;
//                }
//              }
//            }
//            mut_depthImage.unlock();
//          }
//
//          /* check the sonar sensors */
//          float sonarRange;
//          if (currentPose.z < partialTargetPose.z)  // going upward
//          {
//            mut_sonarUpward.lock();
//            sonarRange = sonarUpwardRange;
//            mut_sonarUpward.unlock();
//          }
//          else  // going downward
//          {
//            mut_sonarDownward.lock();
//            sonarRange = sonarDownwardRange;
//            mut_sonarDownward.unlock();
//          }
//
//          // if there is a vertical obstacle
//          if (sonarRange < sonarMaxRange
//              && std::abs(partialTargetPose.z - currentPose.z) > (sonarRange))
//          {
//            // if the quadrotor only needs to move vertically and is obstructed, then we deem the destination is unreachable
//            if (planarDistToTarget <= maxPositionError)
//            {
//              urs_wearable::Pose dest = controller.getDest();
//              dest.x = currentPose.x;
//              dest.y = currentPose.y;
//              if (currentPose.z < partialTargetPose.z)  // going upward
//              {
//                dest.z = currentPose.z + (sonarRange - distFromObsVertical);
//              }
//              else  // going downward
//              {
//                dest.z = currentPose.z - (sonarRange - distFromObsVertical);
//              }
//              controller.setDest(dest, false);
//              std::cout << "Destination unreachable [by the sonar sensor]" << std::endl;
//              break;
//            }
//
//            // devise a new partial target
//            partialTargetPose.x = (partialTargetPose.x + currentPose.x) / 2;
//            partialTargetPose.y = (partialTargetPose.y + currentPose.y) / 2;
//            if (currentPose.z < partialTargetPose.z)  // going upward
//            {
//              partialTargetPose.z = currentPose.z + (sonarRange - distFromObsVertical);
//            }
//            else  // going downward
//            {
//              partialTargetPose.z = currentPose.z - (sonarRange - distFromObsVertical);
//            }
////            std::cout << ns << "S) " << partialTargetPose.x << ", " << partialTargetPose.y  << ", " << partialTargetPose.z << std::endl;
//          }
//
//          /* set the destination of the UAV */
//          urs_wearable::Pose dest = controller.getDest();
//          dest.x = partialTargetPose.x;
//          dest.y = partialTargetPose.y;
//          dest.z = partialTargetPose.z;
//          controller.setDest(dest, false);
//        }
//      }
//
//      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
//    }
//  }
//  catch (boost::thread_interrupted&)
//  {
//    std::cout << "A navigation thread " << ns << " to ["
//        << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has been interrupted" << std::endl;
//  }
//  std::cout << "A navigation thread " << ns << " to ["
//      << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has terminated" << std::endl;
//}

//void Navigator::_navigate(Controller& controller, const urs_wearable::Pose targetPose, const bool oriented)
//{
//  std::cout << "A navigation thread " << ns << " to ["
//      << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has started" << std::endl;
//
//  try
//  {
//    urs_wearable::Pose partialTargetPose;
//    partialTargetPose.x = targetPose.x;
//    partialTargetPose.y = targetPose.y;
//    partialTargetPose.z = targetPose.z;
//
//    bool hasObstructedCenter = false;
//
//    // (int)(std::atan2(0.7, 2.0) * 180.0 / M_PI / 0.25) + 1 = 78
//    // (int)(std::atan2(0.5, 1.0) * 180.0 / M_PI / 0.25) + 1 = 57
//    // (int)(std::atan2(0.7, 1.0) * 180.0 / M_PI / 0.25) + 1 = 140
//    int sideStep = (int)(std::atan2(safetyMargin, 2.0) * 180.0 / M_PI / laserAngularResolution) + 1;
//
//    while (true)
//    {
//      urs_wearable::Pose currentPose = controller.getPose();
//
//      // if the UAV reaches the target destination
//      if (Navigator::getDistance(currentPose, targetPose) <= maxPositionError)
//      {
//        if (!oriented || Navigator::getYawDiff(currentPose.yaw, targetPose.yaw) <= maxOrientationError * M_PI / 180.0) {
//          break;
//        }
//        else
//        {
//          urs_wearable::Pose dest = controller.getDest();
//          dest.yaw = targetPose.yaw;
//          controller.setDest(dest, true);
//        }
//      }
//      else  // otherwise, begin navigating
//      {
//        // update the partial target destination if the UAV has reached it
//        if (Navigator::getDistance(currentPose, partialTargetPose) <= maxPositionError)
//        {
//          if (hasObstructedCenter)
//          {
//            partialTargetPose.x = (targetPose.x + currentPose.x) / 2;
//            partialTargetPose.y = (targetPose.y + currentPose.y) / 2;
//            partialTargetPose.z = currentPose.z + 0.5;
//            hasObstructedCenter = false;
//          }
//          else
//          {
//            partialTargetPose.x = targetPose.x;
//            partialTargetPose.y = targetPose.y;
//            partialTargetPose.z = targetPose.z;
//          }
////            std::cout << ns << "RE) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//        }
//
//        double vx = partialTargetPose.x - currentPose.x;
//        double vy = partialTargetPose.y - currentPose.y;
//        double planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//        double yawHeaded = std::atan2(vy, vx);
//        if (yawHeaded < 0.0)
//        {
//          yawHeaded += M_PI + M_PI;
//        }
//
//        // rotate only if it needs to move horizontally
//        if (planarDistToTarget > maxPositionError)
//        {
//          urs_wearable::Pose dest = controller.getDest();
//          dest.yaw = yawHeaded;
//          controller.setDest(dest, true);
//        }
//
//        // if the quadrotor has aligned itself to the partial target destination
//        if (Navigator::getYawDiff(currentPose.yaw, yawHeaded) <= M_PI / 180.0  // allow 1.0 degree of error
//            || planarDistToTarget <= maxPositionError)              // or only needs to fly vertically
//        {
//          if (planarDistToTarget > maxPositionError)  // if needs to move horizontally
//          {
//            double panDegree = std::atan2(safetyMargin, planarDistToTarget) * 180.0 / M_PI;
//            int panStep = (int)(panDegree / laserAngularResolution) + 1;
//
//            // legal range: 390 <- 540 -> 690. The maximum steps from 540 to both side are 150
//            int panStepTrimmed = (panStep > 150)? 150: panStep;
//
//            // TODO: In the case that panSteps > 150,
//            // there might be a problem if the quadrotor is asked to go through a passage than it can't squeeze through.
//
//            /* check the laser scanner */
//            mut_laser.lock();
//            if (!laserScan.ranges.empty())
//            {
//              bool obstructedCenter = false;
//              bool obstructedLeft = false;
//              bool obstructedRight = false;
//              double obstructedCenterDist = DBL_MAX;
//              double obstructedLeftDist = DBL_MAX;
//              double obstructedRightDist = DBL_MAX;
//
//              if (laserScan.ranges[laserCenterStep] < planarDistToTarget + safetyMargin)
//              {
//                obstructedCenter = true;
//                obstructedCenterDist = laserScan.ranges[laserCenterStep];
//              }
//              double upperLimit = (planarDistToTarget + safetyMargin) / std::cos(panStepTrimmed * laserAngularResolution * M_PI / 180.0);
//              if (laserScan.ranges[laserCenterStep - panStepTrimmed] < upperLimit)
//              {
//                obstructedLeft = true;
//                obstructedLeftDist = laserScan.ranges[laserCenterStep - panStepTrimmed];
//              }
//              else if (laserScan.ranges[laserCenterStep - sideStep] < upperLimit)
//              {
//                obstructedLeft = true;
//                obstructedLeftDist = laserScan.ranges[laserCenterStep - sideStep];
//              }
//
//              if (laserScan.ranges[laserCenterStep + panStepTrimmed] < upperLimit)
//              {
//                obstructedRight = true;
//                obstructedRightDist = laserScan.ranges[laserCenterStep + panStepTrimmed];
//              }
//              else if (laserScan.ranges[laserCenterStep + sideStep] < upperLimit)
//              {
//                obstructedRight = true;
//                obstructedRightDist = laserScan.ranges[laserCenterStep + sideStep];
//              }
//
//              // if the path is blocked, we need to devise a new partial target
//              if (obstructedCenter || obstructedLeft || obstructedRight)
//              {
//                // if the quadrotor is in the same level as the target
//                if (std::fabs(currentPose.z - partialTargetPose.z) <= maxPositionError
//                    && (obstructedLeftDist < 6.0 || obstructedCenterDist < 6.0 || obstructedRightDist < 6.0))
//                {
////                  // if the obstacle is at the (partial) target, then we deem the destination is unreachable
////                  double lowerLimit = (planarDistToTarget - safetyMargin) / std::cos(std::abs(panStepObstructed) * laserAngularResolution * M_PI / 180.0);
////                  if (obstructedDistance > lowerLimit)
////                  {
////                    mut_dest[uavID].lock();
////                    dest[uavID].x = currentPose.x;
////                    dest[uavID].y = currentPose.y;
////                    dest[uavID].z = currentPose.z;
////                    mut_dest[uavID].unlock();
////                    std::cout << "Destination unreachable [by the laser scanner]" << std::endl;
////                    mut_laser[uavID].unlock();
////                    break;
////                  }
//
//                  if (!obstructedLeft)
//                  {
////                    if (laserScan[uavID].ranges[330] > 4.0)
////                    {
////                      double angle = (540 - 330) * laserAngularResolution * M_PI / 180.0;
////                      double c = std::cos(angle);
////                      double s = std::sin(angle);
////                      double vxUnit = vx / planarDistToTarget;
////                      double vyUnit = vy / planarDistToTarget;
////
////                      partialTargetPose.x = currentPose.x + (vxUnit * c - vyUnit * s);
////                      partialTargetPose.y = currentPose.y + (vxUnit * s + vyUnit * c);
////
////                      std::cout << ns << "L1) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
////                      mut_laser[uavID].unlock();
////                      continue;
////                    }
//
//                    if (laserScan.ranges[180] > 2.0)
//                    {
//                      urs_wearable::Pose pose = controller.getPose();
//                      vx = partialTargetPose.x - pose.x;
//                      vy = partialTargetPose.y - pose.y;
//                      planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//                      partialTargetPose.x = pose.x + (-vy / planarDistToTarget);
//                      partialTargetPose.y = pose.y + (vx / planarDistToTarget);
//                      partialTargetPose.z = pose.z + 0.5;
//
////                      partialTargetPose.x = currentPose.x + (-vy / planarDistToTarget);
////                      partialTargetPose.y = currentPose.y + (vx / planarDistToTarget);
//
////                      std::cout << ns << "L2) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//                      mut_laser.unlock();
//                      hasObstructedCenter = false;
//                      continue;
//                    }
//                  }
//
//                  if (!obstructedRight)
//                  {
////                    if (laserScan[uavID].ranges[750] > 4.0)
////                    {
////                      double angle = (540 - 750) * laserAngularResolution * M_PI / 180.0;
////                      double c = std::cos(angle);
////                      double s = std::sin(angle);
////                      double vxUnit = vx / planarDistToTarget;
////                      double vyUnit = vy / planarDistToTarget;
////
////                      partialTargetPose.x = currentPose.x + (vxUnit * c - vyUnit * s);
////                      partialTargetPose.y = currentPose.y + (vxUnit * s + vyUnit * c);
////
////                      std::cout << uavID << "R1) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
////                      mut_laser[uavID].unlock();
////                      continue;
////                    }
//
//                    if (laserScan.ranges[900] > 2.0)
//                    {
//                      urs_wearable::Pose pose = controller.getPose();
//                      vx = partialTargetPose.x - pose.x;
//                      vy = partialTargetPose.y - pose.y;
//                      planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//                      partialTargetPose.x = pose.x + (vy / planarDistToTarget);
//                      partialTargetPose.y = pose.y + (-vx / planarDistToTarget);
//                      partialTargetPose.z = pose.z + 0.5;
//
////                      partialTargetPose.x = currentPose.x + (vy / planarDistToTarget);
////                      partialTargetPose.y = currentPose.y + (-vx / planarDistToTarget);
//
////                      std::cout << ns << "R2) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//                      mut_laser.unlock();
//                      hasObstructedCenter = false;
//                      continue;
//                    }
//                  }
//                }
//
//                // if reach here, it means it hasn't got a new partial target
//                if (obstructedCenterDist < 10.0)
//                {
//                  urs_wearable::Pose pose = controller.getPose();
//                  vx = partialTargetPose.x - pose.x;
//                  vy = partialTargetPose.y - pose.y;
//                  planarDistToTarget = std::sqrt(vx * vx + vy * vy);
//                  partialTargetPose.x = pose.x + (-vx / planarDistToTarget) * (1.0 - obstructedCenterDist);
//                  partialTargetPose.y = pose.y + (-vy / planarDistToTarget) * (1.0 - obstructedCenterDist);
//                  partialTargetPose.z = pose.z + 0.5;
//
////                  partialTargetPose.x = currentPose.x + (-vx / planarDistToTarget) * (1.0 - obstructedCenterDist);
////                  partialTargetPose.y = currentPose.y + (-vy / planarDistToTarget) * (1.0 - obstructedCenterDist);
////                  partialTargetPose.z = currentPose.z + 0.5;
//
////                  std::cout << ns << "CT) " << partialTargetPose.x << ", " << partialTargetPose.y << ", " << partialTargetPose.z << std::endl;
//                  hasObstructedCenter = true;
//                }
//              }
//            }
//            mut_laser.unlock();
//          }
//
//          /* check the sonar sensors */
//          float sonarRange;
//          if (currentPose.z < partialTargetPose.z)  // going upward
//          {
//            mut_sonarUpward.lock();
//            sonarRange = sonarUpwardRange;
//            mut_sonarUpward.unlock();
//          }
//          else  // going downward
//          {
//            mut_sonarDownward.lock();
//            sonarRange = sonarDownwardRange;
//            mut_sonarDownward.unlock();
//          }
//
//          // if there is a vertical obstacle
//          if (sonarRange < sonarMaxRange
//              && std::abs(partialTargetPose.z - currentPose.z) > (sonarRange))
//          {
//            // if the quadrotor only needs to move vertically and is obstructed, then we deem the destination is unreachable
//            if (planarDistToTarget <= maxPositionError)
//            {
//              urs_wearable::Pose dest = controller.getDest();
//              dest.x = currentPose.x;
//              dest.y = currentPose.y;
//              if (currentPose.z < partialTargetPose.z)  // going upward
//              {
//                dest.z = currentPose.z + (sonarRange - distFromObsVertical);
//              }
//              else  // going downward
//              {
//                dest.z = currentPose.z - (sonarRange - distFromObsVertical);
//              }
//              controller.setDest(dest, false);
//              std::cout << "Destination unreachable [by the sonar sensor]" << std::endl;
//              break;
//            }
//
//            // devise a new partial target
//            partialTargetPose.x = (partialTargetPose.x + currentPose.x) / 2;
//            partialTargetPose.y = (partialTargetPose.y + currentPose.y) / 2;
//            if (currentPose.z < partialTargetPose.z)  // going upward
//            {
//              partialTargetPose.z = currentPose.z + (sonarRange - distFromObsVertical);
//            }
//            else  // going downward
//            {
//              partialTargetPose.z = currentPose.z - (sonarRange - distFromObsVertical);
//            }
////            std::cout << ns << "S) " << partialTargetPose.x << ", " << partialTargetPose.y  << ", " << partialTargetPose.z << std::endl;
//          }
//
//          /* set the destination of the UAV */
//          urs_wearable::Pose dest = controller.getDest();
//          dest.x = partialTargetPose.x;
//          dest.y = partialTargetPose.y;
//          dest.z = partialTargetPose.z;
//          controller.setDest(dest, false);
//        }
//      }
//
//      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
//    }
//  }
//  catch (boost::thread_interrupted&)
//  {
//    std::cout << "A navigation thread " << ns << " to ["
//        << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has been interrupted" << std::endl;
//  }
//  std::cout << "A navigation thread " << ns << " to ["
//      << targetPose.x << ", " << targetPose.y << ", " << targetPose.z << "] has terminated" << std::endl;
//}

void Navigator::cancel()
{
  navigationThread.interrupt();

  depthImageSub.shutdown();
//  laserSub.shutdown();
  sonarDownwardSub.shutdown();
  sonarUpwardSub.shutdown();
}

void Navigator::setNamespace(const std::string& ns)
{
  this->ns = ns;
}

//void Navigator::_readLaserScan(const sensor_msgs::LaserScanConstPtr& msg)
//{
//  mut_laser.lock();
//  laserScan = *msg;
//  mut_laser.unlock();
//}

void Navigator::_readSonarDownward(const sensor_msgs::RangeConstPtr& msg)
{
  mut_sonarDownward.lock();
  sonarDownwardRange = msg->range;
  mut_sonarDownward.unlock();
}

void Navigator::_readSonarUpward(const sensor_msgs::RangeConstPtr& msg)
{
  mut_sonarUpward.lock();
  sonarUpwardRange = msg->range;
  mut_sonarUpward.unlock();
}

double Navigator::getDistance(const urs_wearable::PoseEuler& from, const urs_wearable::PoseEuler& to)
{
  return std::sqrt(
      (to.position.x - from.position.x) * (to.position.x - from.position.x)
      + (to.position.y - from.position.y) * (to.position.y - from.position.y)
      + (to.position.z - from.position.z) * (to.position.z - from.position.z));
}

double Navigator::getYawDiff(double yaw1, double yaw2)   // yaw1 and yaw2 are in radian
{
  double diff = std::fabs(yaw1 - yaw2);
  return (diff > M_PI)? M_PI + M_PI - diff: diff;
}
