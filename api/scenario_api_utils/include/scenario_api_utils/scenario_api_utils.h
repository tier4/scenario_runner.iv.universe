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

#pragma once
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;
namespace bt = bg::strategy::transform;
typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::linestring<Point> Line;
typedef bg::model::polygon<Point> Polygon;

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(0.5);
  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }
    rate.sleep();
  }
}

double normalizeRadian(const double rad, const double min_rad = -M_PI, const double max_rad = M_PI);
geometry_msgs::Quaternion quatFromYaw(double yaw);
double yawFromQuat(double q_x, double q_y, double q_z, double q_w);
double yawFromQuat(geometry_msgs::Quaternion q);
geometry_msgs::Pose poseFromValue(
  const double p_x, const double p_y, const double p_z, const double o_x, const double o_y,
  const double o_z, const double o_w);
geometry_msgs::Pose poseFromValue(
  const double p_x, const double p_y, const double p_z, const double yaw);
geometry_msgs::Pose movePose(const geometry_msgs::Pose & pose, const double move_dist_to_forward);

#endif  // SCENARIO_API_SCENARIO_API_UTILS_H_INCLUDED
