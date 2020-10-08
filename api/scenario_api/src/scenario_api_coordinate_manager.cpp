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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <scenario_api/scenario_api_coordinate_manager.h>

ScenarioAPICoordinateManager::ScenarioAPICoordinateManager() {}

ScenarioAPICoordinateManager::~ScenarioAPICoordinateManager() {}

bool ScenarioAPICoordinateManager::setFrameId(
  const std::string frame_id, const geometry_msgs::Pose pose)
{
  // TODO: now, source frame is only map
  if (coordinate_map_.find(frame_id) != coordinate_map_.end()) {
    ROS_WARN("Frame:(id:%s) already exsists", frame_id.c_str());
    return false;
  }

  // input coordinate
  tf2::Transform map2frame;
  tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  tf2::Quaternion orientation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  map2frame.setOrigin(position);
  map2frame.setRotation(orientation);

  // register coordinate
  coordinate_map_[frame_id] = map2frame;

  return true;
}

geometry_msgs::Pose ScenarioAPICoordinateManager::getRelativePose(
  const std::string frame_id, const geometry_msgs::Pose pose)
{
  geometry_msgs::Pose relative_pose;

  if (coordinate_map_.find(frame_id) == coordinate_map_.end()) {
    ROS_WARN("Frame(id:%s) does not exsist", frame_id.c_str());
    return relative_pose;  // TODO return nullptr (change function type)
  }

  tf2::Transform frame2newframe;
  tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  tf2::Quaternion orientation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  frame2newframe.setOrigin(position);
  frame2newframe.setRotation(orientation);

  tf2::Transform map2frame = coordinate_map_[frame_id];
  tf2::Transform map2newframe = map2frame * frame2newframe;
  tf2::toMsg(map2newframe, relative_pose);

  return relative_pose;
}
