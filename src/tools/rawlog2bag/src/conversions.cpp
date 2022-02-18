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

#include <mrpt/math/CVectorFixed.h>
#include <sensor_msgs/NavSatStatus.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "rawlog2bag/conversions.h"

namespace rawlog2bag
{
ros::Time convert(const mrpt::system::TTimeStamp &src)
{
  ros::Time dst;
  const double time = mrpt::system::timestampTotime_t(src);
  dst.sec = static_cast<uint64_t>(time);
  dst.nsec = static_cast<uint64_t>(std::fmod(time, 1.0) * 1e9 + 0.5);
  return dst;
}

geometry_msgs::Pose convert(const mrpt::poses::CPose2D &pose)
{
  geometry_msgs::Pose msg;
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  mrpt::math::CQuaternion<double> orientation;
  auto v = mrpt::math::CVectorFixed<double, 3>::Zero();
  v[2] = pose.phi();
  orientation.fromRodriguesVector(v);
  msg.orientation = convert(orientation);
  return msg;
}

geometry_msgs::Pose convert(const mrpt::poses::CPose3D &pose)
{
  geometry_msgs::Pose msg;
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  msg.position.z = pose.z();
  mrpt::math::CQuaternion<double> orientation;
  pose.getAsQuaternion(orientation);
  msg.orientation = convert(orientation);
  return msg;
}

geometry_msgs::Twist convert(const mrpt::math::TTwist2D &twist)
{
  geometry_msgs::Twist msg;
  msg.linear.x = twist.vx;
  msg.linear.y = twist.vy;
  msg.angular.z = twist.omega;
  return msg;
}

geometry_msgs::TransformStamped convert(
    const mrpt::poses::CPose3D &pose, const mrpt::system::TTimeStamp &timestamp,
    const std::string &target_frame, const std::string &source_frame)
{
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = convert(timestamp);
  msg.header.frame_id = target_frame;
  msg.child_frame_id = source_frame;
  msg.transform.translation.x = pose.x();
  msg.transform.translation.y = pose.y();
  msg.transform.translation.z = pose.z();
  mrpt::math::CQuaternion<double> orientation;
  pose.getAsQuaternion(orientation);
  msg.transform.rotation = convert(orientation);
  return msg;
}

geometry_msgs::TransformStamped convert(
    const mrpt::math::TPose3D &pose, const mrpt::system::TTimeStamp &timestamp,
    const std::string &target_frame, const std::string &source_frame)
{
  return convert(
      mrpt::poses::CPose3D(pose), timestamp,
      target_frame, source_frame);
}

bool convert(
    const mrpt::obs::CObservationOdometry::Ptr &obs,
    const std::string &odom_frame, const std::string &base_frame,
    nav_msgs::Odometry *msg)
{
  msg->header.stamp = convert(obs->timestamp);
  msg->header.frame_id = odom_frame;
  msg->child_frame_id = base_frame;
  msg->pose.pose = convert(obs->odometry);
  msg->pose.covariance[0] = -1;
  if (obs->hasVelocities) {
    msg->twist.twist = convert(obs->velocityLocal);
    msg->twist.covariance[0] = -1;
  }
  return true;
}

bool convert(
    const mrpt::obs::CObservation2DRangeScan::Ptr &obs,
    const std::string &laser_frame, sensor_msgs::LaserScan *msg)
{
  if (obs->getScanSize() == 0) {
    return false;
  }
  msg->header.stamp = convert(obs->timestamp);
  msg->header.frame_id = laser_frame;
  msg->angle_min = -obs->aperture / 2.;
  msg->angle_increment = obs->aperture / (obs->getScanSize() - 1);
  msg->angle_max = obs->aperture / 2.;
  msg->range_max = obs->maxRange;
  constexpr auto inf = std::numeric_limits<float>::infinity();
  msg->ranges.resize(obs->getScanSize(), inf);
  if (obs->hasIntensity()) {
    msg->intensities.resize(obs->getScanSize(), 0.);
  }
  for (size_t i = 0; i < obs->getScanSize(); ++i) {
    size_t j = !obs->rightToLeft ? obs->getScanSize() - i : i;
    if (obs->getScanRangeValidity(j)) {
      msg->ranges[i] = obs->getScanRange(j);
      if (obs->hasIntensity()) {
        msg->intensities[i] = obs->getScanIntensity(j);
      }
    }
  }
  return true;
}

bool convert(
    const mrpt::obs::CObservationIMU::Ptr &obs,
    const std::string &imu_frame, sensor_msgs::Imu *msg)
{
  bool ok = false;
  msg->header.stamp = convert(obs->timestamp);
  msg->header.frame_id = imu_frame;
  if (obs->has(mrpt::obs::IMU_ORI_QUAT_X)) {
    msg->orientation.x = obs->get(mrpt::obs::IMU_ORI_QUAT_X);
    msg->orientation.y = obs->get(mrpt::obs::IMU_ORI_QUAT_Y);
    msg->orientation.z = obs->get(mrpt::obs::IMU_ORI_QUAT_Z);
    msg->orientation.w = obs->get(mrpt::obs::IMU_ORI_QUAT_W);
    ok = true;
  } else if (obs->has(mrpt::obs::IMU_YAW)) {
    mrpt::math::TPose3D pose;
    pose.yaw = obs->get(mrpt::obs::IMU_YAW);
    pose.pitch = obs->get(mrpt::obs::IMU_PITCH);
    pose.roll = obs->get(mrpt::obs::IMU_ROLL);
    mrpt::math::CQuaternion<double> orientation;
    pose.getAsQuaternion(orientation);
    msg->orientation = rawlog2bag::convert(orientation);
    ok = true;
  }
  msg->orientation_covariance[0] = -1.;
  if (obs->has(mrpt::obs::IMU_WZ)) {
    msg->angular_velocity.z = obs->get(mrpt::obs::IMU_WZ);
    msg->angular_velocity.y = obs->get(mrpt::obs::IMU_WY);
    msg->angular_velocity.x = obs->get(mrpt::obs::IMU_WX);
    ok = true;
  }
  msg->angular_velocity_covariance[0] = -1.;
  if (obs->has(mrpt::obs::IMU_X_ACC)) {
    msg->linear_acceleration.x = obs->get(mrpt::obs::IMU_X_ACC);
    msg->linear_acceleration.y = obs->get(mrpt::obs::IMU_Y_ACC);
    msg->linear_acceleration.z = obs->get(mrpt::obs::IMU_Z_ACC);
    ok = true;
  }
  msg->linear_acceleration_covariance[0] = -1.;
  return ok;
}

bool convert(
    const mrpt::obs::CObservationGPS::Ptr &obs,
    const std::string &antenna_frame, sensor_msgs::NavSatFix *msg)
{
  if (!obs->hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>()) {
    return false;
  }
  const mrpt::obs::gnss::Message_NMEA_GGA &gga =
      obs->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
  msg->header.stamp = convert(obs->timestamp);
  msg->header.frame_id = antenna_frame;
  msg->latitude = gga.fields.latitude_degrees;
  msg->longitude = gga.fields.longitude_degrees;
  msg->altitude = gga.fields.altitude_meters;
  msg->position_covariance[0] = -1.;
  msg->position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  switch (gga.fields.fix_quality) {
    case 1:
      msg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      break;
    case 3:
      msg->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case 2:
    case 4:
    case 5:
      msg->status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      break;
    default:
      msg->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      break;
  }
  msg->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  return true;
}
}  // namespace rawlog2bag
