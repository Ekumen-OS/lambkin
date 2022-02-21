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

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/datetime.h>
#include <tf2_msgs/TFMessage.h>

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "rawlog2bag/conversions.h"
#include "rawlog2bag/groundtruth_stream.h"

namespace rawlog2bag
{
namespace
{
class GroundtruthRecord : public Record
{
 public:
  GroundtruthRecord(
      const mrpt::poses::CPose3DPDFGaussian& pose,
      const mrpt::system::TTimeStamp& timestamp,
      const std::string& reference_frame,
      const std::string& target_frame) :
    pose_(pose),
    timestamp_(timestamp),
    reference_frame_(reference_frame),
    target_frame_(target_frame)
  {
  }

  mrpt::system::TTimeStamp timestamp() const override
  {
    return timestamp_;
  }

  void writeback(rosbag::Bag* bag) const override
  {
    const ros::Time stamp = convert(timestamp_);
    {
      geometry_msgs::PoseWithCovarianceStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = reference_frame_;
      msg.pose.pose = convert(pose_.mean);
      auto& covariance = pose_.cov;
      if (covariance.norm() == 0.) {
        for (size_t i = 0; i < 6; ++i) {
          for (size_t j = 0; j < 6; ++j) {
            msg.pose.covariance[i * 6 + j] = covariance(i, j);
          }
        }
      } else {
        msg.pose.covariance[0] = -1.;
      }
      bag->write("/gt/" + target_frame_, msg.header.stamp, msg);
    }
    {
      tf2_msgs::TFMessage msg;
      msg.transforms.push_back(convert(
          pose_.mean, timestamp_,
          reference_frame_, target_frame_));
      bag->write("/tf", stamp, msg);
    }
  }

 private:
  const mrpt::poses::CPose3DPDFGaussian pose_;
  const mrpt::system::TTimeStamp timestamp_;
  const std::string reference_frame_;
  const std::string target_frame_;
};

}  // namespace

GroundtruthInputStream::GroundtruthInputStream(
    const std::string& path,
    const std::string& target_frame,
    const std::string& reference_frame) :
  file_(path.c_str()),
  target_frame_(target_frame),
  reference_frame_(reference_frame)
{
}

bool GroundtruthInputStream::eof() const
{
  return file_.eof();
}

std::unique_ptr<Record> GroundtruthInputStream::read()
{
  std::string line;
  if (!std::getline(file_, line)) {
    return nullptr;
  }
  std::istringstream istr{line};
  double ts, x, y, z, yaw, pitch, roll;
  istr >> ts >> x >> y >> z >> yaw >> pitch >> roll;
  auto timestamp = mrpt::system::time_tToTimestamp(ts);
  mrpt::poses::CPose3D mean(x, y, z, yaw, pitch, roll);
  mrpt::poses::CPose3DPDFGaussian pose(mean);
  if (!istr.eof()) {
    auto& covariance = pose.cov;
    istr >> covariance(0, 0) >> covariance(0, 1) >> covariance(0, 2)
        >> covariance(0, 3) >> covariance(0, 4) >> covariance(0, 5)
        >> covariance(1, 1) >> covariance(1, 2) >> covariance(1, 3)
        >> covariance(1, 4) >> covariance(1, 5) >> covariance(2, 2)
        >> covariance(2, 3) >> covariance(2, 4) >> covariance(2, 5)
        >> covariance(3, 3) >> covariance(3, 4) >> covariance(3, 5)
        >> covariance(4, 4) >> covariance(4, 5) >> covariance(5, 5);
    covariance(1, 0) = covariance(0, 1);
    covariance(2, 0) = covariance(0, 2);
    covariance(3, 0) = covariance(0, 3);
    covariance(4, 0) = covariance(0, 4);
    covariance(5, 0) = covariance(0, 5);
    covariance(2, 1) = covariance(1, 2);
    covariance(3, 1) = covariance(1, 3);
    covariance(4, 1) = covariance(1, 4);
    covariance(5, 1) = covariance(1, 5);
    covariance(3, 2) = covariance(2, 3);
    covariance(4, 2) = covariance(2, 4);
    covariance(5, 2) = covariance(2, 5);
    covariance(4, 3) = covariance(3, 4);
    covariance(5, 3) = covariance(3, 5);
    covariance(5, 4) = covariance(4, 5);
  }
  return std::make_unique<GroundtruthRecord>(
      pose, timestamp, reference_frame_, target_frame_);
}

}  // namespace rawlog2bag
