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

/**
 * @file
 * @brief Projects an external 6-DoF odometry estimate into planar odometry.
 */

#include <nav_msgs/msg/odometry.hpp>

namespace external_vio_adapter {

/**
 * @brief Converts a 6-DoF odometry message to a planar odometry message.
 *
 * The x/y position and yaw are retained in the pose. The x/y linear velocity
 * and z angular velocity are retained in the twist. The covariance blocks for
 * `[x, y, yaw]` and `[vx, vy, wz]` are preserved, including their cross terms.
 * Unobserved z, roll, and pitch components receive a large diagonal variance
 * so downstream consumers do not interpret them as certain zeros.
 *
 * @param input External odometry, normally published by OpenVINS on `odomimu`.
 * @param unobserved_axis_variance Variance assigned to discarded axes.
 * @return A planar odometry message with the original timestamp and frames.
 * @throws std::invalid_argument if the pose, quaternion, or covariance is not finite.
 */
nav_msgs::msg::Odometry ProjectToPlanar(const nav_msgs::msg::Odometry& input, double unobserved_axis_variance);

}  // namespace external_vio_adapter
