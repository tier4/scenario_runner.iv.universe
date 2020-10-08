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

#pragma once
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <unordered_map>
class ScenarioAPICoordinateManager
{
public:
  /**
   * @brief constructor
   */
  ScenarioAPICoordinateManager();

  /*
   * @brief destructor
   */
  ~ScenarioAPICoordinateManager();

  // coordinate API
  bool setFrameId(const std::string frame_id, const geometry_msgs::Pose);
  geometry_msgs::Pose getRelativePose(const std::string frame_id, const geometry_msgs::Pose pose);

private:
  std::unordered_map<std::string, tf2::Transform> coordinate_map_;
};