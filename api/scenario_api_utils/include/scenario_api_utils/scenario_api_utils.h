/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SCENARIO_API_SCENARIO_API_UTILS_H_INCLUDED
#define SCENARIO_API_SCENARIO_API_UTILS_H_INCLUDED

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

template <class T>
T waitForParam(const std::shared_ptr<rclcpp::Node> & node, const std::string & key)
{
  T value;
  // TODO: replace this with Rate that works based on ROS Clock when it is available
  // https://github.com/ros2/rclcpp/pull/1373
  rclcpp::WallRate rate(0.5);
  while (!rclcpp::ok()) {
    try{
      const auto result = node->declare_parameter(key).get<T>();
      return result;
    } catch(...)
    {
      rate.sleep();
    }
  }
}

double normalizeRadian(const double rad, const double min_rad = -M_PI, const double max_rad = M_PI);
geometry_msgs::msg::Quaternion quatFromYaw(double yaw);
double yawFromQuat(double q_x, double q_y, double q_z, double q_w);
double yawFromQuat(geometry_msgs::msg::Quaternion q);
geometry_msgs::msg::Pose poseFromValue(
  const double p_x, const double p_y, const double p_z, const double o_x, const double o_y,
  const double o_z, const double o_w);
geometry_msgs::msg::Pose poseFromValue(
  const double p_x, const double p_y, const double p_z, const double yaw);
geometry_msgs::msg::Pose movePose(const geometry_msgs::msg::Pose & pose, const double move_dist_to_forward);

#endif  // SCENARIO_API_SCENARIO_API_UTILS_H_INCLUDED
