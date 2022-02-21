// Copyright 2022 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RAWLOG2BAG_CONVERSIONS_H
#define RAWLOG2BAG_CONVERSIONS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/datetime.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>

#include <string>

namespace rawlog2bag
{
template <typename T>
inline geometry_msgs::Quaternion
convert(const mrpt::math::CQuaternion<T> &q)
{
  geometry_msgs::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.r();
  return msg;
}

ros::Time convert(const mrpt::system::TTimeStamp &src);

geometry_msgs::Pose convert(const mrpt::poses::CPose2D &pose);

geometry_msgs::Pose convert(const mrpt::poses::CPose3D &pose);

geometry_msgs::Twist convert(const mrpt::math::TTwist2D &twist);

geometry_msgs::TransformStamped convert(
    const mrpt::math::TPose3D &pose, const mrpt::system::TTimeStamp &timestamp,
    const std::string &target_frame, const std::string &source_frame);

geometry_msgs::TransformStamped convert(
    const mrpt::poses::CPose3D &pose, const mrpt::system::TTimeStamp &timestamp,
    const std::string &target_frame, const std::string &source_frame);

bool convert(
    const mrpt::obs::CObservationOdometry::Ptr &obs,
    const std::string &odom_frame,
    const std::string &base_frame,
    nav_msgs::Odometry *msg);

bool convert(
    const mrpt::obs::CObservation2DRangeScan::Ptr &obs,
    const std::string &laser_frame, sensor_msgs::LaserScan *msg);

bool convert(
    const mrpt::obs::CObservationIMU::Ptr &obs,
    const std::string &imu_frame, sensor_msgs::Imu *msg);

bool convert(
    const mrpt::obs::CObservationGPS::Ptr &obs,
    const std::string &antenna_frame, sensor_msgs::NavSatFix *msg);

}  // namespace rawlog2bag

#endif  // RAWLOG2BAG_CONVERSIONS_H
