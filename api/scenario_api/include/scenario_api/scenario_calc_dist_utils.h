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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <scenario_api_utils/scenario_api_utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/utils.h>

double calcDistFromPolygonToPointCloud(
  const std::shared_ptr<sensor_msgs::PointCloud2> & pointcloud_ptr, const Polygon poly,
  const bool consider_height, const double top, const double bottom);

Polygon makeRelativePolygonFromSelf(
  const geometry_msgs::Pose self_pose, const geometry_msgs::Pose obj_pose,
  const geometry_msgs::Vector3 obj_size);

double calcDistOfPolygon(const Polygon poly, const Polygon poly2);

#endif  // SCENARIO_API_SCENARIO_API_CALC_DIST_UTILS_H_INCLUDED
