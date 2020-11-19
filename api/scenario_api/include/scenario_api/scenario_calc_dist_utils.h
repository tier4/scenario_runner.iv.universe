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

#ifndef SCENARIO_API_SCENARIO_API_CALC_DIST_UTILS_H_INCLUDED
#define SCENARIO_API_SCENARIO_API_CALC_DIST_UTILS_H_INCLUDED

#include <scenario_api_autoware/scenario_api_autoware.h>
#include <scenario_api_utils/scenario_api_utils.h>
#include <tf2/utils.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace scenario_api_utils
{
double calcDistFromPolygonToPointCloud(
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> & pointcloud_ptr,
  const ScenarioAPIAutoware::Polygon & poly, const bool consider_height, const double top,
  const double bottom, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

ScenarioAPIAutoware::Polygon makeRelativePolygonFromSelf(
  const geometry_msgs::msg::Pose self_pose, const geometry_msgs::msg::Pose obj_pose,
  const geometry_msgs::msg::Vector3 obj_size);

double calcDistOfPolygon(
  const ScenarioAPIAutoware::Polygon & poly, const ScenarioAPIAutoware::Polygon & poly2);
}  // namespace scenario_api_utils
#endif  // SCENARIO_API_SCENARIO_API_CALC_DIST_UTILS_H_INCLUDED
