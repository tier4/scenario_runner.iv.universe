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

#include <scenario_api/scenario_api_core.h>

ScenarioAPI::ScenarioAPI()
{
  /* construct API class*/
  autoware_api_ = std::make_shared<ScenarioAPIAutoware>();
  simulator_api_ = std::make_shared<ScenarioAPISimulator>();
  coordinate_api_ = std::make_shared<ScenarioAPICoordinateManager>();
}

ScenarioAPI::~ScenarioAPI() {}

// basic API
bool ScenarioAPI::setEgoCarName(const std::string & name)
{
  ego_car_name_ = name;
  return true;
}

bool ScenarioAPI::isEgoCarName(const std::string & name) { return ego_car_name_ == name; }

bool ScenarioAPI::isAPIReady()
{
  if (!autoware_api_->isAPIReady()) {
    return false;
  }
  if (!simulator_api_->isAPIReady()) {
    return false;
  }
  return true;
}

bool ScenarioAPI::waitAPIReady() { return autoware_api_->waitAPIReady(); }

bool ScenarioAPI::updateState() { return simulator_api_->updateState(); }

// start API
bool ScenarioAPI::sendStartPoint(
  const geometry_msgs::Pose pose, const bool wait_ready, const std::string & frame_type)
{
  if (!simulator_api_->spawnStartPoint(pose)) return false;
  if (!autoware_api_->sendStartPoint(pose, wait_ready, frame_type)) return false;
  return true;
}

bool ScenarioAPI::sendGoalPoint(
  const std::string & name, const geometry_msgs::Pose pose, const bool wait_ready,
  const std::string & frame_type)
{
  if (name == ego_car_name_) {
    //ego-car
    return autoware_api_->sendGoalPoint(pose, wait_ready, frame_type);
  } else {
    //npc-object
    return simulator_api_->sendNPCToGoalPoint(name, pose, wait_ready, frame_type);
  }
}

bool ScenarioAPI::sendCheckPoint(
  const std::string & name, const geometry_msgs::Pose pose, const bool wait_ready,
  const std::string & frame_type)
{
  if (name == ego_car_name_) {
    return autoware_api_->sendCheckPoint(pose, wait_ready, frame_type);
  } else {
    return simulator_api_->sendNPCToCheckPoint(name, pose, wait_ready, frame_type);
  }
}

bool ScenarioAPI::sendStartVelocity(const double velocity)
{
  autoware_api_->sendStartVelocity(velocity);
  //sleep for autoware path plan
  //*if you engage autoware soon after using this API,
  //*egp-vehicle decelerates after engage.
  ros::Rate(1.0).sleep();
}

bool ScenarioAPI::sendEngage(const bool engage)
{
  autoware_api_->sendEngage(engage);
  simulator_api_->sendEngage(engage);
}

bool ScenarioAPI::waitAutowareInitialize() { return autoware_api_->waitAutowareInitialize(); }

bool ScenarioAPI::setMaxSpeed(double velocity) { return autoware_api_->setMaxSpeed(velocity); }

// coordinate API
bool ScenarioAPI::setFrameId(std::string frame_id, const geometry_msgs::Pose pose)
{
  return coordinate_api_->setFrameId(frame_id, pose);
}

geometry_msgs::Pose ScenarioAPI::getRelativePose(
  std::string frame_id, const geometry_msgs::Pose pose)
{
  return coordinate_api_->getRelativePose(frame_id, pose);
}

// self-vehicle API
Pose2D ScenarioAPI::getCurrentPose() { return autoware_api_->getCurrentPose(); }

geometry_msgs::PoseStamped ScenarioAPI::getCurrentPoseRos()
{
  return autoware_api_->getCurrentPoseRos();
}

double ScenarioAPI::getVelocity() { return autoware_api_->getVelocity(); }

double ScenarioAPI::getAccel() { return autoware_api_->getAccel(); }

double ScenarioAPI::getJerk() { return autoware_api_->getJerk(); }

double ScenarioAPI::getMoveDistance() { return autoware_api_->getMoveDistance(); }

bool ScenarioAPI::isStopped(double thresh_velocity)
{
  return (std::abs(getVelocity()) <= thresh_velocity);
}

bool ScenarioAPI::isInArea(geometry_msgs::Pose pose, double dist_thresh, double delta_yaw_thresh)
{
  double yaw = yawFromQuat(pose.orientation);
  double delta_yaw = std::abs(normalizeRadian(getCurrentPose().yaw - yaw));
  double delta_dist =
    std::hypot(getCurrentPose().x - pose.position.x, getCurrentPose().y - pose.position.y);
  return (delta_dist < dist_thresh) and (delta_yaw < delta_yaw_thresh);
}

bool ScenarioAPI::willLaneChange() { return autoware_api_->willLaneChange(); }

bool ScenarioAPI::getLeftBlinker() { return autoware_api_->getLeftBlinker(); }
bool ScenarioAPI::getRightBlinker() { return autoware_api_->getRightBlinker(); }

bool ScenarioAPI::approveLaneChange(bool approve_lane_change)
{
  return autoware_api_->approveLaneChange(approve_lane_change);
}

double ScenarioAPI::getMinimumDistanceToObstacle(bool consider_height)
{
  const std::shared_ptr<sensor_msgs::PointCloud2> pcl_ptr =
    autoware_api_->getPointCloud();  // TODO ->move to sensor_api_->getPointCloud();
  return calcMinimumDistanceToObstacle(pcl_ptr, consider_height);
}

double ScenarioAPI::calcMinimumDistanceToObstacle(
  const std::shared_ptr<sensor_msgs::PointCloud2> & pointcloud_ptr, const bool consider_height)
{
  // rename
  const double top = autoware_api_->getVehicleTopFromBase();
  const double bottom = autoware_api_->getVehicleBottomFromBase();
  Polygon poly = autoware_api_->getSelfPolygon2D();
  return calcDistFromPolygonToPointCloud(pointcloud_ptr, poly, consider_height, top, bottom);
}

bool ScenarioAPI::getCurrentLaneID(int & current_id, double max_dist, double max_delta_yaw)
{
  return autoware_api_->getCurrentLaneID(current_id, max_dist, max_delta_yaw);
}

bool ScenarioAPI::isChangeLaneID() { return autoware_api_->isChangeLaneID(); }

bool ScenarioAPI::getDistancefromCenterLine(double & dist_from_center_line)
{
  return autoware_api_->getDistancefromCenterLine(dist_from_center_line);
}

bool ScenarioAPI::isInLane() { return autoware_api_->isInLane(); }

// NPC API
bool ScenarioAPI::addNPC(
  const std::string & npc_type, const std::string & name, geometry_msgs::Pose pose,
  const double velocity, const bool stop_by_vehicle, const std::string & frame_type)
{
  return simulator_api_->addNPC(npc_type, name, pose, velocity, stop_by_vehicle, frame_type);
}

bool ScenarioAPI::changeNPCVelocity(const std::string & name, const double velocity)
{
  return simulator_api_->changeNPCVelocity(name, velocity);
}

bool ScenarioAPI::changeNPCAccelMin(const std::string & name, const double accel)
{
  return simulator_api_->changeNPCAccelMin(name, accel);
}

bool ScenarioAPI::changeNPCAccelMax(const std::string & name, const double accel)
{
  return simulator_api_->changeNPCAccelMax(name, accel);
}

bool ScenarioAPI::changeNPCVelocityWithAccel(
  const std::string & name, const double velocity, const double accel)
{
  return simulator_api_->changeNPCVelocityWithAccel(name, velocity, accel);
}

bool ScenarioAPI::changeNPCConsiderVehicle(
  const std::string & name, const bool consider_ego_vehicle)
{
  return simulator_api_->changeNPCConsiderVehicle(name, consider_ego_vehicle);
}

bool ScenarioAPI::changeNPCLaneChangeLeft(const std::string & name)
{
  return simulator_api_->changeNPCLaneChangeLeft(name);
}

bool ScenarioAPI::changeNPCLaneChangeRight(const std::string & name)
{
  return simulator_api_->changeNPCLaneChangeRight(name);
}

bool ScenarioAPI::changeNPCLaneChange(const std::string & name, const int target_lane_id)
{
  return simulator_api_->changeNPCLaneChange(name, target_lane_id);
}

bool ScenarioAPI::changeNPCUturn(const std::string & name)
{
  return simulator_api_->changeNPCUturn(name);
}

bool ScenarioAPI::changeNPCTurnLeft(const std::string & name)
{
  return simulator_api_->changeNPCTurnLeft(name);
}

bool ScenarioAPI::changeNPCTurnRight(const std::string & name)
{
  return simulator_api_->changeNPCTurnRight(name);
}

bool ScenarioAPI::changeNPCNoTurn(const std::string & name)
{
  return simulator_api_->changeNPCNoTurn(name);
}

bool ScenarioAPI::changeNPCIgnoreLane(const std::string & name)
{
  return simulator_api_->changeNPCIgnoreLane(name);
}

bool ScenarioAPI::deleteNPC(const std::string & name) { return simulator_api_->deleteNPC(name); }

bool ScenarioAPI::getNPC(
  const std::string & name, geometry_msgs::Pose & object_pose, geometry_msgs::Twist & object_twist,
  geometry_msgs::Vector3 & object_size, std::string & object_name)
{
  return simulator_api_->getNPC(name, object_pose, object_twist, object_size, object_name);
}

bool ScenarioAPI::calcDistToNPC(double & dist_to_npc, const std::string & name)
{
  geometry_msgs::Pose obj_pose;
  geometry_msgs::Twist obj_twist;
  geometry_msgs::Vector3 obj_size;
  std::string obj_name;

  if (!getNPC(name, obj_pose, obj_twist, obj_size, obj_name)) return false;

  // create polygon
  Polygon self_poly = autoware_api_->getSelfPolygon2D();
  Polygon obj_poly = makeRelativePolygonFromSelf(getCurrentPoseRos().pose, obj_pose, obj_size);

  dist_to_npc = calcDistOfPolygon(self_poly, obj_poly);
  return true;
}

bool ScenarioAPI::calcDistToNPCFromNPC(
  double & distance, const std::string & npc1_name, const std::string & npc2_name)
{
  geometry_msgs::Pose npc1_pose, npc2_pose;

  geometry_msgs::Twist npc1_twist, npc2_twist;

  geometry_msgs::Vector3 npc1_size, npc2_size;

  std::string unused;

  if (not getNPC(npc1_name, npc1_pose, npc1_twist, npc1_size, unused)) {
    return false;
  }

  if (not getNPC(npc2_name, npc2_pose, npc2_twist, npc2_size, unused)) {
    return false;
  }

  const auto npc1_polygon{
    makeRelativePolygonFromSelf(getCurrentPoseRos().pose, npc1_pose, npc1_size)};

  const auto npc2_polygon{
    makeRelativePolygonFromSelf(getCurrentPoseRos().pose, npc2_pose, npc2_size)};

  return not std::isnan(distance = calcDistOfPolygon(npc1_polygon, npc2_polygon));
}

bool ScenarioAPI::finishNPCLaneChange(const std::string & name, bool * finish_lane_change)
{
  return simulator_api_->checkNPCFinishLaneChange(name, *finish_lane_change);
}

bool ScenarioAPI::finishNPCVelocityChange(const std::string & name, bool * finish_velocity_change)
{
  return simulator_api_->checkNPCFinishVelocityChange(name, *finish_velocity_change);
}

bool ScenarioAPI::getNPCVelocity(const std::string name, double * velocity)
{
  return simulator_api_->getNPCVelocity(name, velocity);
}

bool ScenarioAPI::getNPCAccel(const std::string name, double * accel)
{
  return simulator_api_->getNPCAccel(name, accel);
}

std::vector<std::string> ScenarioAPI::getNpcList() { return simulator_api_->getNpcList(); }

bool ScenarioAPI::isNpcExist(const std::string & name) { return true; }

// traffic light API
bool ScenarioAPI::setTrafficLightColor(
  const int traffic_id, const std::string traffic_color, const bool use_traffic_light)
{
  if (use_traffic_light) {
    //TODO not implemented yet
    // send simulator to traffic signal color
    return false;
  } else {
    // send autoware to detection result of traffic signal
    return autoware_api_->setTrafficLightsColor(traffic_id, traffic_color);
  }
}

bool ScenarioAPI::setTrafficLightArrow(
  const int traffic_id, const std::string traffic_arrow, const bool use_traffic_light)
{
  if (use_traffic_light) {
    //TODO not implemented yet
    // send simulator to traffic signal arrow
    return false;
  } else {
    // send autoware to detection result of traffic signal
    return autoware_api_->setTrafficLightsArrow(traffic_id, traffic_arrow);
  }
}

bool ScenarioAPI::resetTrafficLightColor(const int traffic_id, const bool use_traffic_light)
{
  if (use_traffic_light) {
    //TODO not implemented yet
    // reset traffic light of simulator
    return false;
  } else {
    // reset detection result(only color)
    return autoware_api_->resetTrafficLightsColor(traffic_id);
  }
}

bool ScenarioAPI::resetTrafficLightArrow(const int traffic_id, const bool use_traffic_light)
{
  if (use_traffic_light) {
    //TODO not implemented yet
    // reset traffic arrow of simulator
    return false;
  } else {
    // reset detection result(only arrow)
    return autoware_api_->resetTrafficLightsArrow(traffic_id);
  }
}

bool ScenarioAPI::getTrafficLightColor(
  const int traffic_id, std::string * traffic_color, const bool use_traffic_light)
{
  if (use_traffic_light) {
    //TODO not implemented yet
    // send simulator to traffic signal color
    return false;
  } else {
    // get detection result of traffic signal color from autoware
    return autoware_api_->getTrafficLightColor(traffic_id, traffic_color);
  }
}

bool ScenarioAPI::getTrafficLightArrow(
  const int traffic_id, std::vector<std::string> * const traffic_arrow,
  const bool use_traffic_light)
{
  if (use_traffic_light) {
    //TODO not implemented yet
    // send simulator to traffic signal arrow
    return false;
  } else {
    // get detection result of traffic signal arrow from autoware
    return autoware_api_->getTrafficLightArrow(traffic_id, traffic_arrow);
  }
}

bool ScenarioAPI::getTrafficLineCenterPose(
  const int traffic_relation_id, geometry_msgs::Pose & line_pose)
{
  return autoware_api_->getTrafficLineCenterPose(traffic_relation_id, line_pose);
}

bool ScenarioAPI::getDistanceToTrafficLight(const int traffic_relation_id, double & distance)
{
  auto self_pose = autoware_api_->getCurrentPoseRos();
  return autoware_api_->getDistanceToTrafficLight(traffic_relation_id, self_pose.pose, distance);
}

bool ScenarioAPI::getDistanceToTrafficLine(const int traffic_relation_id, double & distance)
{
  auto self_pose = autoware_api_->getCurrentPoseRos();
  return autoware_api_->getDistanceToTrafficLine(traffic_relation_id, self_pose.pose, distance);
}

bool ScenarioAPI::checkOverTrafficLine(const int traffic_relation_id, bool & over_line)
{
  auto self_pose = autoware_api_->getCurrentPoseRos();
  return autoware_api_->checkOverTrafficLine(traffic_relation_id, self_pose.pose, over_line);
}

double ScenarioAPI::getDistanceToArea(
  const std::string & name, const geometry_msgs::Pose pose, const std::string & frame_type)
{
  // get position of target object and shift pose
  geometry_msgs::Pose obj_pose;
  geometry_msgs::Pose shift_pose;
  if (name == ego_car_name_) { // ego-car
    obj_pose = autoware_api_->getCurrentPoseRos().pose;
    if (!autoware_api_->shiftEgoPose(pose, frame_type, &shift_pose)) {
      return false;
    }
  } else { // npc-object
    npc_simulator::Object obj;
    if (!simulator_api_->getNPC(name, obj)) {
      return false;
    }
    obj_pose = obj.initial_state.pose_covariance.pose;
    if (!simulator_api_->shiftNPCPose(pose, frame_type, obj, &shift_pose)) {
      return false;
    }
  }
  return std::sqrt(
    std::pow(obj_pose.position.x - shift_pose.position.x, 2) +
    std::pow(obj_pose.position.y - shift_pose.position.y, 2));
}

//util API
bool ScenarioAPI::isObjectInArea(
  const std::string & name, geometry_msgs::Pose pose, double dist_thresh, double delta_yaw_thresh,
  const std::string & frame_type)
{
  // get position of target object and shift pose
  geometry_msgs::Pose obj_pose;
  geometry_msgs::Pose shift_pose;
  if (name == ego_car_name_) {
    //ego-car
    obj_pose = autoware_api_->getCurrentPoseRos().pose;
    if (!autoware_api_->shiftEgoPose(pose, frame_type, &shift_pose)) {
      return false;
    }
  } else {
    //npc-object
    npc_simulator::Object obj;
    if (!simulator_api_->getNPC(name, obj)) {
      return false;
    }
    obj_pose = obj.initial_state.pose_covariance.pose;
    if (!simulator_api_->shiftNPCPose(pose, frame_type, obj, &shift_pose)) {
      return false;
    }
  }
  const auto npc_yaw = yawFromQuat(obj_pose.orientation);
  const auto yaw = yawFromQuat(shift_pose.orientation);
  double delta_yaw = std::abs(normalizeRadian(npc_yaw - yaw));
  double delta_dist = sqrt(
    std::pow(obj_pose.position.x - shift_pose.position.x, 2) +
    std::pow(obj_pose.position.y - shift_pose.position.y, 2));
  return (delta_dist < dist_thresh) and (delta_yaw < delta_yaw_thresh);
}

geometry_msgs::Pose ScenarioAPI::genPoseROS(
  const double x, const double y, const double z, const double yaw)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = quatFromYaw(yaw);
  return pose;
}
geometry_msgs::Pose ScenarioAPI::genPoseROS(
  const double p_x, const double p_y, const double p_z, const double o_x, const double o_y,
  const double o_z, const double o_w)
{
  geometry_msgs::Pose pose;
  pose.position.x = p_x;
  pose.position.y = p_y;
  pose.position.z = p_z;
  pose.orientation.x = o_x;
  pose.orientation.y = o_y;
  pose.orientation.z = o_z;
  pose.orientation.w = o_w;
  return pose;
}
