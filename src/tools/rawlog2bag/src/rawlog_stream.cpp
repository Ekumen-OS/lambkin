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

#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>

#include <memory>
#include <string>
#include <utility>

#include "rawlog2bag/conversions.h"
#include "rawlog2bag/rawlog_stream.h"

namespace rawlog2bag
{
namespace
{
std::string lowercase(std::string str)
{
  std::transform(
      str.begin(), str.end(), str.begin(),
      [](char c) { return std::tolower(c); });
  return str;
}

class RawlogRecord : public Record
{
 public:
  RawlogRecord(
      mrpt::obs::CObservation::Ptr obs,
      const std::string& base_frame) :
    obs_(obs), base_frame_(base_frame)
  {
  }

  mrpt::system::TTimeStamp timestamp() const override
  {
    return obs_->timestamp;
  }

  void writeback(rosbag::Bag* bag) const override
  {
    const std::string sensor_name = lowercase(obs_->sensorLabel);
    const std::string sensor_frame = sensor_name + "_link";
    if (obs_->GetRuntimeClass() == CLASS_ID(mrpt::obs::CObservationOdometry)) {
      mrpt::obs::CObservationOdometry::Ptr obs =
          std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(obs_);
      nav_msgs::Odometry msg;
      if (convert(obs, sensor_frame, base_frame_, &msg)) {
        bag->write("/odom/" + sensor_name, msg.header.stamp, msg);
      }
    } else if (obs_->GetRuntimeClass() == CLASS_ID(mrpt::obs::CObservation2DRangeScan)) {
      mrpt::obs::CObservation2DRangeScan::Ptr obs =
          std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(obs_);
      sensor_msgs::LaserScan msg;
      if (convert(obs, sensor_frame, &msg)) {
        bag->write("/scan/" + sensor_name, msg.header.stamp, msg);
      }
    } else if (obs_->GetRuntimeClass() == CLASS_ID(mrpt::obs::CObservationIMU)) {
      mrpt::obs::CObservationIMU::Ptr obs =
          std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obs_);
      sensor_msgs::Imu msg;
      if (convert(obs, sensor_frame, &msg)) {
        bag->write("/imu/" + sensor_name, msg.header.stamp, msg);
      }
    } else if (obs_->GetRuntimeClass() == CLASS_ID(mrpt::obs::CObservationGPS)) {
      mrpt::obs::CObservationGPS::Ptr obs =
          std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(obs_);
      sensor_msgs::NavSatFix msg;
      if (convert(obs, sensor_frame, &msg)) {
        bag->write("/fix/" + sensor_name, msg.header.stamp, msg);
      }
    }
    mrpt::math::TPose3D pose;
    obs_->getSensorPose(pose);
    tf2_msgs::TFMessage msg;
    msg.transforms.push_back(convert(
        pose, obs_->timestamp, base_frame_, sensor_frame));
    bag->write("/tf", convert(obs_->timestamp), msg);
  }

 private:
  mrpt::obs::CObservation::Ptr obs_;
  const std::string base_frame_;
};

}  // namespace

RawlogInputStream::RawlogInputStream(
    const std::string& path,
    const std::string& base_frame,
    bool ignore_unknown) :
  base_frame_(base_frame),
  ignore_unknown_(ignore_unknown)
{
  if (!rawlog_.loadFromRawLogFile(path)) {
    throw std::runtime_error("Failed to open '" + path + "'");
  }
}

bool RawlogInputStream::eof() const
{
  return index_ >= rawlog_.size();
}

std::unique_ptr<Record> RawlogInputStream::read()
{
  if (index_ < rawlog_.size()) {
    size_t index = index_++;
    switch (rawlog_.getType(index)) {
      case mrpt::obs::CRawlog::etObservation:
        return std::make_unique<RawlogRecord>(
            rawlog_.getAsObservation(index),
            base_frame_);
      default:
        if (!ignore_unknown_) {
          throw std::runtime_error("Found unknown record");
        }
        break;
    }
  }
  return nullptr;
}

}  // namespace rawlog2bag
