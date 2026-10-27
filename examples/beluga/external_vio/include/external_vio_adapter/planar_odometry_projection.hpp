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

#include <nav_msgs/msg/odometry.hpp>

namespace external_vio_adapter {

/// Projects a six-degree-of-freedom odometry estimate into planar odometry.
nav_msgs::msg::Odometry ProjectToPlanar(
    const nav_msgs::msg::Odometry& input,
    double unobserved_axis_variance);

}  // namespace external_vio_adapter
