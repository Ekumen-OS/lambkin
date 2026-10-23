// Copyright 2026 Ekumen, Inc.
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

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <external_vio_adapter/planar_odometry_projection.hpp>

namespace external_vio_adapter {
namespace {

constexpr std::array<std::size_t, 3> kPlanarIndices{0, 1, 5};
constexpr double kQuaternionNormEpsilon = 1e-12;

bool IsFinite(const nav_msgs::msg::Odometry& message) {
  const auto& pose = message.pose.pose;
  const auto& twist = message.twist.twist;
  const std::array<double, 13> values{pose.position.x,    pose.position.y,    pose.position.z,    pose.orientation.x,
                                      pose.orientation.y, pose.orientation.z, pose.orientation.w, twist.linear.x,
                                      twist.linear.y,     twist.linear.z,     twist.angular.x,    twist.angular.y,
                                      twist.angular.z};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : message.pose.covariance) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : message.twist.covariance) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

void ProjectCovariance(
    const std::array<double, 36>& input,
    std::array<double, 36>& output,
    double unobserved_axis_variance) {
  output.fill(0.0);
  for (std::size_t row = 0; row < kPlanarIndices.size(); ++row) {
    for (std::size_t column = 0; column < kPlanarIndices.size(); ++column) {
      output[6 * kPlanarIndices[row] + kPlanarIndices[column]] =
          input[6 * kPlanarIndices[row] + kPlanarIndices[column]];
    }
  }
  for (const std::size_t index : std::array<std::size_t, 3>{2, 3, 4}) {
    output[6 * index + index] = unobserved_axis_variance;
  }
}

}  // namespace

nav_msgs::msg::Odometry ProjectToPlanar(const nav_msgs::msg::Odometry& input, double unobserved_axis_variance) {
  if (!IsFinite(input)) {
    throw std::invalid_argument("External odometry contains a non-finite value");
  }
  if (!std::isfinite(unobserved_axis_variance) || unobserved_axis_variance <= 0.0) {
    throw std::invalid_argument("Unobserved-axis variance must be finite and positive");
  }

  const auto& input_orientation = input.pose.pose.orientation;
  const double quaternion_norm = std::sqrt(
      input_orientation.x * input_orientation.x + input_orientation.y * input_orientation.y +
      input_orientation.z * input_orientation.z + input_orientation.w * input_orientation.w);
  if (quaternion_norm <= kQuaternionNormEpsilon) {
    throw std::invalid_argument("External odometry contains a zero quaternion");
  }

  const double x = input_orientation.x / quaternion_norm;
  const double y = input_orientation.y / quaternion_norm;
  const double z = input_orientation.z / quaternion_norm;
  const double w = input_orientation.w / quaternion_norm;
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  auto output = input;
  output.pose.pose.position.z = 0.0;
  output.pose.pose.orientation.x = 0.0;
  output.pose.pose.orientation.y = 0.0;
  output.pose.pose.orientation.z = std::sin(yaw / 2.0);
  output.pose.pose.orientation.w = std::cos(yaw / 2.0);
  output.twist.twist.linear.z = 0.0;
  output.twist.twist.angular.x = 0.0;
  output.twist.twist.angular.y = 0.0;

  ProjectCovariance(input.pose.covariance, output.pose.covariance, unobserved_axis_variance);
  ProjectCovariance(input.twist.covariance, output.twist.covariance, unobserved_axis_variance);
  return output;
}

}  // namespace external_vio_adapter
