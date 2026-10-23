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
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>

#include <external_vio_adapter/planar_odometry_adapter.hpp>
#include <external_vio_adapter/planar_odometry_projection.hpp>

namespace {

tf2::Transform ToTfTransform(const geometry_msgs::msg::Pose& pose) {
  return tf2::Transform(
      tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
      tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

tf2::Transform ToTfTransform(const geometry_msgs::msg::Transform& transform) {
  return tf2::Transform(
      tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
      tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));
}

void SetRosPose(const tf2::Transform& transform, geometry_msgs::msg::Pose& pose) {
  const auto& origin = transform.getOrigin();
  const auto rotation = transform.getRotation();
  pose.position.x = origin.x();
  pose.position.y = origin.y();
  pose.position.z = origin.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
}

std::array<double, 36> TransformTwistCovariance(
    const std::array<double, 36>& covariance,
    const tf2::Transform& base_from_body) {
  std::array<double, 36> result{};
  const auto rotation = base_from_body.getBasis();
  const auto lever_arm = base_from_body.getOrigin();

  double jacobian[6][6]{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      jacobian[row][column] = rotation[row][column];
      jacobian[row + 3][column + 3] = rotation[row][column];
      jacobian[row][column + 3] = 0.0;
    }
  }
  const tf2::Matrix3x3 lever_arm_cross(
      0.0, -lever_arm.z(), lever_arm.y(), lever_arm.z(), 0.0, -lever_arm.x(), -lever_arm.y(), lever_arm.x(), 0.0);
  const auto lever_arm_rotation = lever_arm_cross * rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      jacobian[row][column + 3] = lever_arm_rotation[row][column];
    }
  }

  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      double value = 0.0;
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          value += jacobian[row][i] * covariance[6 * i + j] * jacobian[column][j];
        }
      }
      result[6 * row + column] = value;
    }
  }
  return result;
}

std::array<double, 6> ToPoseVector(const tf2::Transform& transform) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  const auto& origin = transform.getOrigin();
  return {origin.x(), origin.y(), origin.z(), roll, pitch, yaw};
}

tf2::Transform FromPoseVector(const std::array<double, 6>& pose) {
  tf2::Quaternion rotation;
  rotation.setRPY(pose[3], pose[4], pose[5]);
  return tf2::Transform(rotation, tf2::Vector3(pose[0], pose[1], pose[2]));
}

double WrappedAngleDifference(double lhs, double rhs) {
  return std::remainder(lhs - rhs, 2.0 * 3.14159265358979323846);
}

std::array<double, 36> TransformPoseCovariance(
    const geometry_msgs::msg::Pose& input_pose,
    const std::array<double, 36>& covariance,
    const tf2::Transform& base_from_body) {
  constexpr double kFiniteDifferenceStep = 1e-6;
  const auto input_vector = ToPoseVector(ToTfTransform(input_pose));
  const auto nominal = ToPoseVector(ToTfTransform(input_pose) * base_from_body.inverse());
  double jacobian[6][6]{};
  for (std::size_t column = 0; column < 6; ++column) {
    auto perturbed = input_vector;
    perturbed[column] += kFiniteDifferenceStep;
    const auto transformed = ToPoseVector(FromPoseVector(perturbed) * base_from_body.inverse());
    for (std::size_t row = 0; row < 6; ++row) {
      const double difference =
          row < 3 ? transformed[row] - nominal[row] : WrappedAngleDifference(transformed[row], nominal[row]);
      jacobian[row][column] = difference / kFiniteDifferenceStep;
    }
  }

  std::array<double, 36> result{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      double value = 0.0;
      for (std::size_t i = 0; i < 6; ++i) {
        for (std::size_t j = 0; j < 6; ++j) {
          value += jacobian[row][i] * covariance[6 * i + j] * jacobian[column][j];
        }
      }
      result[6 * row + column] = value;
    }
  }
  return result;
}

nav_msgs::msg::Odometry TransformToBase(
    const nav_msgs::msg::Odometry& input,
    const geometry_msgs::msg::TransformStamped& transform_stamped,
    std::string_view base_frame) {
  const auto base_from_body = ToTfTransform(transform_stamped.transform);
  const auto world_from_body = ToTfTransform(input.pose.pose);
  // T_world_base = T_world_body * T_base_body^-1 because the TF maps body coordinates into base coordinates.
  const auto world_from_base = world_from_body * base_from_body.inverse();

  auto output = input;
  SetRosPose(world_from_base, output.pose.pose);
  output.child_frame_id = std::string(base_frame);
  output.pose.covariance = TransformPoseCovariance(input.pose.pose, input.pose.covariance, base_from_body);

  const auto& input_twist = input.twist.twist;
  const auto rotation = base_from_body.getBasis();
  const auto lever_arm = base_from_body.getOrigin();
  const tf2::Vector3 body_linear_velocity(input_twist.linear.x, input_twist.linear.y, input_twist.linear.z);
  const tf2::Vector3 body_angular_velocity(input_twist.angular.x, input_twist.angular.y, input_twist.angular.z);
  const auto base_angular_velocity = rotation * body_angular_velocity;
  const auto base_linear_velocity = rotation * body_linear_velocity + lever_arm.cross(base_angular_velocity);

  output.twist.twist.linear.x = base_linear_velocity.x();
  output.twist.twist.linear.y = base_linear_velocity.y();
  output.twist.twist.linear.z = base_linear_velocity.z();
  output.twist.twist.angular.x = base_angular_velocity.x();
  output.twist.twist.angular.y = base_angular_velocity.y();
  output.twist.twist.angular.z = base_angular_velocity.z();
  output.twist.covariance = TransformTwistCovariance(input.twist.covariance, base_from_body);
  return output;
}

}  // namespace

namespace external_vio_adapter {

PlanarOdometryAdapter::PlanarOdometryAdapter(const rclcpp::NodeOptions& options)
    : rclcpp::Node("planar_odometry_adapter", options),
      unobserved_axis_variance_(declare_parameter<double>("unobserved_axis_variance", 1e6)),
      base_frame_(declare_parameter<std::string>("base_frame", "")),
      transform_timeout_(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(declare_parameter<double>("transform_timeout", 0.05)))) {
  const auto input_topic = declare_parameter<std::string>("input_topic", "/ov_msckf/odomimu");
  const auto output_topic = declare_parameter<std::string>("output_topic", "/beluga/vio/odometry");
  const auto queue_size = declare_parameter<int>("queue_size", 10);
  if (queue_size < 1) {
    throw std::invalid_argument("queue_size must be positive");
  }
  if (transform_timeout_.count() < 0) {
    throw std::invalid_argument("transform_timeout must not be negative");
  }

  if (!base_frame_.empty()) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  }

  const auto qos = rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(queue_size));
  publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, qos);
  subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { OnOdometry(std::move(message)); });

  RCLCPP_INFO(
      get_logger(), "Converting external odometry '%s' to planar output '%s'", input_topic.c_str(),
      output_topic.c_str());
  if (!base_frame_.empty()) {
    RCLCPP_INFO(
        get_logger(), "Transforming odometry body frame into base frame '%s' before projection", base_frame_.c_str());
  }
}

void PlanarOdometryAdapter::OnOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message) {
  if (message->header.frame_id.empty() || message->child_frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping odometry without both frame_id and child_frame_id");
    return;
  }

  try {
    auto odometry = *message;
    if (!base_frame_.empty() && message->child_frame_id != base_frame_) {
      const auto transform = tf_buffer_->lookupTransform(
          base_frame_, message->child_frame_id, message->header.stamp,
          rclcpp::Duration::from_nanoseconds(transform_timeout_.count()));
      odometry = TransformToBase(*message, transform, base_frame_);
    }
    publisher_->publish(ProjectToPlanar(odometry, unobserved_axis_variance_));
  } catch (const tf2::TransformException& error) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping odometry without base-frame transform: %s", error.what());
  } catch (const std::invalid_argument& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping invalid external odometry: %s", error.what());
  }
}

}  // namespace external_vio_adapter
