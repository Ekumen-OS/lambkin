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

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace external_vio_adapter {

/**
 * @brief ROS 2 adapter from OpenVINS odometry to planar Beluga odometry.
 */
class PlanarOdometryAdapter final : public rclcpp::Node {
 public:
  /// @brief Constructs the adapter and declares its ROS parameters.
  explicit PlanarOdometryAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void OnOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message);

  double unobserved_axis_variance_;
  std::string base_frame_;
  std::chrono::nanoseconds transform_timeout_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace external_vio_adapter
