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

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include <external_vio_adapter/planar_odometry_projection.hpp>

namespace external_vio_adapter {
namespace {

TEST(PlanarOdometryProjection, RetainsPlanarPoseAndTwist) {
  nav_msgs::msg::Odometry input;
  input.header.frame_id = "global";
  input.child_frame_id = "imu";
  input.pose.pose.position.x = 1.5;
  input.pose.pose.position.y = -0.2;
  input.pose.pose.position.z = 4.0;
  input.pose.pose.orientation.z = std::sin(0.4);
  input.pose.pose.orientation.w = std::cos(0.4);
  input.twist.twist.linear.x = 2.0;
  input.twist.twist.linear.y = 0.3;
  input.twist.twist.linear.z = 1.0;
  input.twist.twist.angular.x = 0.5;
  input.twist.twist.angular.y = 0.6;
  input.twist.twist.angular.z = 0.7;
  input.pose.covariance[0] = 1.0;
  input.pose.covariance[1] = 0.1;
  input.pose.covariance[5] = 0.2;
  input.pose.covariance[30] = 0.2;
  input.pose.covariance[35] = 0.3;
  input.twist.covariance[0] = 0.4;
  input.twist.covariance[35] = 0.5;

  const auto output = ProjectToPlanar(input, 1e6);

  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.y, -0.2);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.z, 0.0);
  EXPECT_NEAR(output.pose.pose.orientation.z, std::sin(0.4), 1e-12);
  EXPECT_NEAR(output.pose.pose.orientation.w, std::cos(0.4), 1e-12);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 2.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.y, 0.3);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(output.twist.twist.angular.z, 0.7);
  EXPECT_DOUBLE_EQ(output.pose.covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(output.pose.covariance[1], 0.1);
  EXPECT_DOUBLE_EQ(output.pose.covariance[5], 0.2);
  EXPECT_DOUBLE_EQ(output.pose.covariance[35], 0.3);
  EXPECT_DOUBLE_EQ(output.pose.covariance[14], 1e6);
  EXPECT_DOUBLE_EQ(output.twist.covariance[35], 0.5);
}

TEST(PlanarOdometryProjection, RejectsInvalidInput) {
  nav_msgs::msg::Odometry input;
  input.pose.pose.orientation.w = 1.0;
  input.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(ProjectToPlanar(input, 1e6), std::invalid_argument);
}

TEST(PlanarOdometryProjection, RejectsZeroQuaternion) {
  nav_msgs::msg::Odometry input;
  input.pose.pose.orientation.x = 0.0;
  input.pose.pose.orientation.y = 0.0;
  input.pose.pose.orientation.z = 0.0;
  input.pose.pose.orientation.w = 0.0;

  EXPECT_THROW(ProjectToPlanar(input, 1e6), std::invalid_argument);
}

}  // namespace
}  // namespace external_vio_adapter
