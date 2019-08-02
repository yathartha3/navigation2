/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_INTERFACE__LOCAL_PLANNER_BASE_HPP_
#define NAV2_INTERFACE__LOCAL_PLANNER_BASE_HPP_

#include <memory>
#include <string>

#include "nav2_interface/common_types.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

namespace nav2_interface
{

/**
 * @class LocalPlannerInterface
 * @brief planner interface that will be inherited by all local planners
 */
class LocalPlannerInterface : public nav2_util::LifecycleHelperInterface
{
public:
  /**
   * @brief Constructor that brings up pluginlib loaders
   */
  LocalPlannerInterface(
    nav2_util::LifecycleNode::SharedPtr node, TFBufferPtr tf, CostmapROSPtr costmap_ros);

  /**
   * @brief Virtual destructor
   */
  virtual ~LocalPlannerInterface() {}

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Sets the global goal for this local planner.
   * @param goal_pose The Goal Pose
   */
  virtual void setGoalPose(const nav_2d_msgs::Pose2DStamped& goal_pose) = 0;

  /**
   * @brief local setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_2d_msgs::msg::Path2D & path);

  /**
   * @brief local_planner_base computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @return The best command for the robot to drive
   */
  nav_2d_msgs::msg::Twist2DStamped computeVelocityCommands(
    const nav_2d_msgs::msg::Pose2DStamped & pose,
    const nav_2d_msgs::msg::Twist2D & velocity);

  /**
   * @brief local_planner_base isGoalReached - Check whether the robot has reached its goal, given the current pose & velocity.
   *
   * The pose that it checks against is the last pose in the current global plan.
   * The calculation is delegated to the goal_checker plugin.
   *
   * @param pose Current pose
   * @param velocity Current velocity
   * @return True if the robot should be considered as having reached the goal.
   */
  bool isGoalReached(
    const nav_2d_msgs::msg::Pose2DStamped & pose,
    const nav_2d_msgs::msg::Twist2D & velocity);
};

}  // namespace local_planner_base

#endif  // NAV2_INTERFACE__LOCAL_PLANNER_BASE_HPP_