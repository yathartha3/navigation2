// Copyright (c) 2018 Intel Corporation, 2019 Samsung Research America
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
#include <chrono>
#include <ctime>
#include <thread>
#include <algorithm>
#include <memory>

#include "spin.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace nav2_recoveries
{

Spin::Spin()
: Recovery<SpinAction>()
{
  initial_yaw_ = 0.0;
}

Spin::~Spin()
{
}

void Spin::onConfigure()
{
  nav2_util::declare_parameter_if_not_declared(node_,
    "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node_->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  nav2_util::declare_parameter_if_not_declared(node_,
    "max_rotational_vel", rclcpp::ParameterValue(1.0));
  node_->get_parameter("max_rotational_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(node_,
    "min_rotational_vel", rclcpp::ParameterValue(0.4));
  node_->get_parameter("min_rotational_vel", min_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(node_,
    "rotational_acc_lim", rclcpp::ParameterValue(3.2));
  node_->get_parameter("rotational_acc_lim", rotational_acc_lim_);
}

Status Spin::onRun(const std::shared_ptr<const SpinAction::Goal> command)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, "odom")) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return Status::FAILED;
  }

  initial_yaw_ = tf2::getYaw(current_pose.pose.orientation);

  cmd_yaw_ = -command->target_yaw;
  RCLCPP_INFO(node_->get_logger(), "Turning %0.2f for spin recovery.",
    cmd_yaw_);
  return Status::SUCCEEDED;
}

Status Spin::onCycleUpdate()
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, "odom")) {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return Status::FAILED;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double relative_yaw = abs(current_yaw - initial_yaw_);
  if (relative_yaw > M_PI) {
    relative_yaw -= 2.0 * M_PI;
  }
  relative_yaw = abs(relative_yaw);

  if (relative_yaw >= abs(cmd_yaw_)) {
    stopRobot();
    return Status::SUCCEEDED;
  }

  double vel = sqrt(2 * rotational_acc_lim_ * relative_yaw);
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_yaw_ < 0 ? cmd_vel.angular.z = -vel : cmd_vel.angular.z = vel;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(relative_yaw, cmd_vel, pose2d)) {
    stopRobot();
    RCLCPP_WARN(node_->get_logger(), "Collision Ahead - Exiting Spin");
    return Status::SUCCEEDED;
  }

  vel_pub_->publish(cmd_vel);

  return Status::RUNNING;
}

bool Spin::isCollisionFree(
  const double & relative_yaw,
  const geometry_msgs::msg::Twist & cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  // Simulate ahead by simulate_ahead_time_ in cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change;
  const int max_cycle_count = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);

  while (cycle_count < max_cycle_count) {
    sim_position_change = cmd_vel.angular.z * (cycle_count / cycle_frequency_);
    pose2d.theta += sim_position_change;
    cycle_count++;

    if (abs(relative_yaw) - abs(sim_position_change) <= 0.) {
      break;
    }

    if (!collision_checker_->isCollisionFree(pose2d)) {
      return false;
    }
  }
  return true;
}

}  // namespace nav2_recoveries

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_recoveries::Spin, nav2_core::Recovery)
