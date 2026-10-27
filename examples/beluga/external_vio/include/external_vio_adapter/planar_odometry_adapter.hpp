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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace external_vio_adapter {

/// Transforms odometry from a body frame into the target base frame.
nav_msgs::msg::Odometry TransformToBase(
    const nav_msgs::msg::Odometry& input,
    const geometry_msgs::msg::TransformStamped& base_from_body);

/// Transforms body-frame odometry and projects it into planar odometry.
nav_msgs::msg::Odometry AdaptToPlanar(
    const nav_msgs::msg::Odometry& input,
    const geometry_msgs::msg::TransformStamped& base_from_body,
    double unobserved_axis_variance);

}  // namespace external_vio_adapter
