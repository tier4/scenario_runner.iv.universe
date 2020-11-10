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

#include <scenario_api_autoware/scenario_api_autoware.h>
#include <scenario_logger/simple_logger.hpp>

ScenarioAPIAutoware::ScenarioAPIAutoware()
: nh_(""),
  pnh_("~"),
  tf_listener_(tf_buffer_),
  is_autoware_ready_initialize(false),
  is_autoware_ready_routing(false),
  total_move_distance_(0.0)
{
  /* Get Parameter*/
  pnh_.param<std::string>(        "camera_frame_id",       camera_frame_id_, "camera_link");
  LOG_SIMPLE(info() << "Parameter 'camera_frame_id' = " << camera_frame_id_);

  //parameter for getMoveDistance
  pnh_.param<bool>(               "rosparam/add_simulator_noise",                         add_simulator_noise_, true);
  LOG_SIMPLE(info() << "Parameter 'rosparam/add_simulator_noise' = " << std::boolalpha << add_simulator_noise_);

  pnh_.param<double>(             "rosparam/simulator_pos_noise",       simulator_noise_pos_dev_, 0.1);
  LOG_SIMPLE(info() << "Parameter 'rosparam/simulator_pos_noise' = " << simulator_noise_pos_dev_);

  pnh_.param<double>(             "rosparam/max_velocity",       autoware_max_velocity_, 30.0);
  LOG_SIMPLE(info() << "Parameter 'rosparam/max_velocity' = " << autoware_max_velocity_);

  /* Scenario parameters*/
  vehicle_data_.wheel_radius = waitForParam<double>(pnh_, "/vehicle_info/wheel_radius");
  LOG_SIMPLE(info() << "Parameter '/vehicle_info/wheel_radius' = " << vehicle_data_.wheel_radius);

  vehicle_data_.wheel_width = waitForParam<double>(pnh_, "/vehicle_info/wheel_width");
  LOG_SIMPLE(info() << "Parameter '/vehicle_info/wheel_width' = " << vehicle_data_.wheel_width);

  vehicle_data_.wheel_base = waitForParam<double>(pnh_, "/vehicle_info/wheel_base");
  LOG_SIMPLE(info() << "Parameter '/vehicle_info/wheel_base' = " << vehicle_data_.wheel_base);

  vehicle_data_.wheel_tread = waitForParam<double>(pnh_, "/vehicle_info/wheel_tread");
  LOG_SIMPLE(info() << "Parameter '/vehicle_info/wheel_tread' = " << vehicle_data_.wheel_tread);

  vehicle_data_.front_overhang = waitForParam<double>(pnh_, "/vehicle_info/front_overhang");
  LOG_SIMPLE(info() << "Parameter '/vehicle_info/front_overhang' = " << vehicle_data_.front_overhang);

  vehicle_data_.rear_overhang = waitForParam<double>(pnh_, "/vehicle_info/rear_overhang");
  LOG_SIMPLE(info() << "Parameter '/vehicle_info/rear_overhang' = " << vehicle_data_.rear_overhang);

  vehicle_data_.vehicle_height = waitForParam<double>(pnh_, "/vehicle_info/vehicle_height");
  LOG_SIMPLE(info() << "Parameter '/vehicle_info/vehicle_height' = " << vehicle_data_.vehicle_height);

  /* register callback*/
  sub_pcl_ = pnh_.subscribe("input/pointcloud", 1, &ScenarioAPIAutoware::callbackPointCloud, this);
  LOG_SIMPLE(info() << "Register callback for topic 'input/pointcloud'");

  sub_map_ = pnh_.subscribe("input/vectormap", 10, &ScenarioAPIAutoware::callbackMap, this);
  LOG_SIMPLE(info() << "Register callback for topic 'input/vectormap'");

  sub_route_ = pnh_.subscribe("input/route", 1, &ScenarioAPIAutoware::callbackRoute, this);
  LOG_SIMPLE(info() << "Register callback for topic 'input/route'");

  sub_state_ =
    pnh_.subscribe("input/autoware_state", 1, &ScenarioAPIAutoware::callbackStatus, this);
  LOG_SIMPLE(info() << "Register callback for topic 'input/autoware_state'");

  sub_twist_ = pnh_.subscribe("input/vehicle_twist", 1, &ScenarioAPIAutoware::callbackTwist, this);
  LOG_SIMPLE(info() << "Register callback for topic 'input/vehicle_twist'");

  sub_turn_signal_ =
    pnh_.subscribe("input/signal_command", 1, &ScenarioAPIAutoware::callbackTurnSignal, this);
  LOG_SIMPLE(info() << "Register callback for topic 'input/signal_command'");

  timer_control_fast_ = pnh_.createTimer(
    ros::Duration(fast_time_control_dt_), &ScenarioAPIAutoware::timerCallbackFast, this);

  timer_control_slow_ = pnh_.createTimer(
    ros::Duration(slow_time_control_dt_), &ScenarioAPIAutoware::timerCallbackSlow, this);

  /* register publisher */
  pub_start_point_ =
    pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("output/start_point", 1, true);
  LOG_SIMPLE(info() << "Advertise topic 'output/start_point'");

  pub_goal_point_ = pnh_.advertise<geometry_msgs::PoseStamped>("output/goal_point", 1, true);
  LOG_SIMPLE(info() << "Advertise topic 'output/goal_point'");

  pub_check_point_ = pnh_.advertise<geometry_msgs::PoseStamped>("output/check_point", 10, true);
  LOG_SIMPLE(info() << "Advertise topic 'output/check_point'");

  pub_start_velocity_ =
    pnh_.advertise<geometry_msgs::TwistStamped>("output/initial_velocity", 1, true);
  LOG_SIMPLE(info() << "Advertise topic 'output/initial_velocity'");

  pub_autoware_engage_ = pnh_.advertise<std_msgs::Bool>("output/autoware_engage", 1, true);
  LOG_SIMPLE(info() << "Advertise topic 'output/autoware_engage'");

  pub_max_velocity_ = pnh_.advertise<std_msgs::Float32>("output/limit_velocity", 1, true);
  LOG_SIMPLE(info() << "Advertise topic 'output/limit_velocity'");

  pub_traffic_detection_result_ =
    pnh_.advertise<autoware_perception_msgs::TrafficLightStateArray>(
      "output/traffic_detection_result", 10, true);
  LOG_SIMPLE(info() << "Advertise topic 'output/traffic_detection_result'");

  pub_lane_change_permission_ =
    pnh_.advertise<std_msgs::Bool>("output/lane_change_permission", 1, true);
  LOG_SIMPLE(info() << "Advertise topic 'output/lane_change_permission'");
}

ScenarioAPIAutoware::~ScenarioAPIAutoware() {}

// callback function

void ScenarioAPIAutoware::timerCallbackFast(const ros::TimerEvent & te)
{
  ros::spinOnce();
  getCurrentPoseFromTF();
  pubTrafficLight();
}

void ScenarioAPIAutoware::timerCallbackSlow(const ros::TimerEvent & te)
{
  updateTotalMoveDistance();
}

void ScenarioAPIAutoware::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  pcl_ptr_ = std::make_shared<sensor_msgs::PointCloud2>(*msg);
}

void ScenarioAPIAutoware::callbackMap(const autoware_lanelet2_msgs::MapBin & msg)
{
  LOG_SIMPLE(info() << "Receive autoware_lanelet2_msgs::MapBin");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

void ScenarioAPIAutoware::callbackRoute(const autoware_planning_msgs::Route & msg)
{
  LOG_SIMPLE(info() << "Receive autoware_planning_msgs::Route");
  is_autoware_ready_routing = true;  // check autoware rady
}

void ScenarioAPIAutoware::callbackStatus(const autoware_system_msgs::AutowareState & msg)
{
  autoware_state_ = msg.state;

  if (autoware_state_ != autoware_system_msgs::AutowareState::Emergency) {
    is_autoware_ready_initialize = true;
  }

  LOG_TOGGLE("Autoware state", autoware_state_);
}

void ScenarioAPIAutoware::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  if (previous_twist_ptr_ != nullptr) {
    second_previous_twist_ptr_ =
      std::make_shared<geometry_msgs::TwistStamped>(*previous_twist_ptr_);
  }

  if (current_twist_ptr_ != nullptr) {
    previous_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*current_twist_ptr_);
  }

  current_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
}

void ScenarioAPIAutoware::callbackTurnSignal(
  const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg)
{
  turn_signal_ptr_ = std::make_shared<autoware_vehicle_msgs::TurnSignal>(*msg);
}

// basic API
bool ScenarioAPIAutoware::isAPIReady()
{
  if (current_pose_ptr_ == nullptr) {
    return false;
  } else {
    LOG_SIMPLE_ONCE(info() << "Ready 'current_pose'");
  }

  if (pcl_ptr_ == nullptr) {
    return false;
  } else {
    LOG_SIMPLE_ONCE(info() << "Ready 'pointcloud'");
  }

  if (lanelet_map_ptr_ == nullptr) {
    return false;
  } else {
    LOG_SIMPLE_ONCE(info() << "Ready 'lanelet2_map'");
  }

  if (current_twist_ptr_ == nullptr) {
    return false;
  } else {
    LOG_SIMPLE_ONCE(info() << "Ready 'current_twist'");
  }

  if (previous_twist_ptr_ == nullptr) {
    return false;
  } else {
    LOG_SIMPLE_ONCE(info() << "Ready 'previous_twist'");
  }

  if (second_previous_twist_ptr_ == nullptr) {
    return false;
  } else {
    LOG_SIMPLE_ONCE(info() << "Ready 'second_previous_twist'");
  }

  if (turn_signal_ptr_ == nullptr) {
    return false;
  } else {
    LOG_SIMPLE_ONCE(info() << "Ready 'turn_signal'");
  }

  return true;
}

bool ScenarioAPIAutoware::waitAPIReady()
{
  while (ros::ok()) {
    LOG_SIMPLE(info() << "Connect to Autoware");
    if (isAPIReady()) {
      LOG_SIMPLE(info() << "Established connection to Autoware");
      break;
    }
    ros::Rate(10.0).sleep();
    ros::spinOnce();
  }
  return true;  // TODO set timeout
}

// start API
bool ScenarioAPIAutoware::sendStartPoint(
  const geometry_msgs::Pose pose, const bool wait_autoware_status, const std::string & frame_type)
{
  LOG_SIMPLE(info() << "Send start-point to Autoware");

  // ignore roll/pitch information
  const double yaw = yawFromQuat(pose.orientation);
  geometry_msgs::PoseWithCovarianceStamped posewcs;
  posewcs.header.stamp = ros::Time::now();
  posewcs.header.frame_id = "map";

  //get pose with frame_type
  geometry_msgs::Pose original_pose;
  original_pose.position = pose.position;
  original_pose.orientation = quatFromYaw(yaw);
  if (!shiftEgoPose(original_pose, frame_type, &posewcs.pose.pose)) {
    return false;
  }

  pub_start_point_.publish(posewcs);

  //publish recurssively until self-pose tf is published
  while (!current_pose_ptr_) {
    ros::Rate(2.0).sleep();
    pub_start_point_.publish(posewcs);
    ros::spinOnce();
  }

  if (wait_autoware_status) {
    //publish recurssively until state changes
    while (!checkState(autoware_system_msgs::AutowareState::WaitingForRoute)) {
      ros::Rate(2.0).sleep();
      posewcs.header.stamp = ros::Time::now();
      pub_start_point_.publish(posewcs);
      ros::spinOnce();
    }
  }
  //In some cases, goalpoint is send soon after startpoint, mission planner cannot plan route.
  sleep(1.0);  //TODO remove this

  return true;  // TODO check success(add timeout function)
}

bool ScenarioAPIAutoware::sendGoalPoint(
  const geometry_msgs::Pose pose, const bool wait_autoware_status, const std::string & frame_type)
{
  // ignore roll/pitch information
  const double yaw = yawFromQuat(pose.orientation);
  geometry_msgs::PoseStamped posestmp;
  posestmp.header.stamp = ros::Time::now();
  posestmp.header.frame_id = "map";

  //get pose with frame_type
  geometry_msgs::Pose original_pose;
  original_pose.position = pose.position;
  original_pose.orientation = quatFromYaw(yaw);
  if (!shiftEgoPose(original_pose, frame_type, &posestmp.pose)) {
    return false;
  }

  pub_goal_point_.publish(posestmp);

  if (wait_autoware_status) {
    //publish recurssively until state changes
    while (!checkState(autoware_system_msgs::AutowareState::WaitingForEngage)) {
      ros::Rate(1.0).sleep();
      posestmp.header.stamp = ros::Time::now();
      pub_goal_point_.publish(posestmp);
      ros::spinOnce();
    }
  }
  //sleep for initial velocity start
  //if there are no sleep from sendgoal to engage,
  //ego vehicle decelerates soon after start
  ros::Rate(1.0).sleep();

  return true;  // TODO check success(add timeout function)
}

bool ScenarioAPIAutoware::sendCheckPoint(
  const geometry_msgs::Pose pose, const bool wait_autoware_status, const std::string & frame_type)
{
  // ignore roll/pitch information
  const double yaw = yawFromQuat(pose.orientation);
  geometry_msgs::PoseStamped posestmp;
  posestmp.header.stamp = ros::Time::now();
  posestmp.header.frame_id = "map";

  //get pose with frame_type
  geometry_msgs::Pose original_pose;
  original_pose.position = pose.position;
  original_pose.orientation = quatFromYaw(yaw);
  if (!shiftEgoPose(original_pose, frame_type, &posestmp.pose)) {
    return false;
  }

  pub_check_point_.publish(posestmp);
  if (wait_autoware_status) {
    // wait for message-received and planning
    waitState(autoware_system_msgs::AutowareState::WaitingForEngage);
  }
  return true;  // TODO check success(add timeout function)
}

bool ScenarioAPIAutoware::checkState(const std::string state)
{
  ROS_INFO_STREAM("autoware_state:" << autoware_state_ << ", target_state" << state << std::endl);
  return autoware_state_ == state;
}

bool ScenarioAPIAutoware::waitState(const std::string state)
{
  // wait for state change
  while (ros::ok()) {
    if (autoware_state_ == state) {
      break;
    }
    ros::Rate(10.0).sleep();
    ros::spinOnce();
  }
  return true;  // TODO: set timeout
}

bool ScenarioAPIAutoware::sendStartVelocity(const double velocity)
{
  LOG_SIMPLE(info() << "Send initial-velocity " << velocity << " to Autoware");

  geometry_msgs::TwistStamped twistmsg;
  twistmsg.header.frame_id = "base_link";
  twistmsg.header.stamp = ros::Time::now();
  twistmsg.twist.linear.x = velocity;
  pub_start_velocity_.publish(twistmsg);
  return true;  // TODO check success
}

bool ScenarioAPIAutoware::sendEngage(const bool engage)
{
  std_msgs::Bool boolmsg;
  boolmsg.data = engage;
  pub_autoware_engage_.publish(boolmsg);
  return true;
}

bool ScenarioAPIAutoware::waitAutowareInitialize()
{
  while (ros::ok()) {
    LOG_SIMPLE(info() << "Waiting for Autoware (Current state is Emergency)");
    if (isAutowareReadyInitialize()) {
      return true;
    }
    ros::Rate(10.0).sleep();
    ros::spinOnce();
  }
  return false;
}

bool ScenarioAPIAutoware::isAutowareReadyInitialize() { return is_autoware_ready_initialize; }

bool ScenarioAPIAutoware::isAutowareReadyRouting()
{
  return is_autoware_ready_routing;
}  // TODO remove this(unused)

bool ScenarioAPIAutoware::setMaxSpeed(double velocity)
{
  LOG_SIMPLE(info() << "Sending max-speed " << velocity << " to Autoware");

  std_msgs::Float32 floatmsg;
  floatmsg.data = velocity;
  pub_max_velocity_.publish(floatmsg);
  return true;  // TODO check success
}

// self-vehicle API
void ScenarioAPIAutoware::getCurrentPoseFromTF(void)
{
  geometry_msgs::TransformStamped transform;
  geometry_msgs::PoseStamped ps;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN_DELAYED_THROTTLE(5.0, "cannot get map to base_link transform. %s", ex.what());
    return;
  }

  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(ps);
}

Pose2D ScenarioAPIAutoware::getCurrentPose()
{
  Pose2D p;
  p.x = current_pose_ptr_->pose.position.x;
  p.y = current_pose_ptr_->pose.position.y;
  p.yaw = tf2::getYaw(current_pose_ptr_->pose.orientation);
  return p;
}

geometry_msgs::PoseStamped ScenarioAPIAutoware::getCurrentPoseRos() { return *current_pose_ptr_; }

Polygon ScenarioAPIAutoware::getSelfPolygon2D() { return getSelfPolygon2D(vehicle_data_); }

double ScenarioAPIAutoware::getVehicleTopFromBase() { return vehicle_data_.top_from_base(); }

double ScenarioAPIAutoware::getVehicleBottomFromBase() { return vehicle_data_.bottom_from_base(); }

double ScenarioAPIAutoware::getVelocity() { return current_twist_ptr_->twist.linear.x; }

double ScenarioAPIAutoware::getAccel() { return getAccel(current_twist_ptr_, previous_twist_ptr_); }

double ScenarioAPIAutoware::getJerk()
{
  return getJerk(current_twist_ptr_, previous_twist_ptr_, second_previous_twist_ptr_);
}

double ScenarioAPIAutoware::getMoveDistance() { return total_move_distance_; }

double ScenarioAPIAutoware::getAccel(
  const std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr,
  const std::shared_ptr<geometry_msgs::TwistStamped> previous_twist_ptr)
{
  double c_x = current_twist_ptr->twist.linear.x;
  double p_x = previous_twist_ptr->twist.linear.x;
  double c_time = current_twist_ptr->header.stamp.toSec();
  double p_time = previous_twist_ptr->header.stamp.toSec();
  if (c_time - p_time <= 0) {
    ROS_WARN("return invalid accel because of invalid timestamp(twist_ptr)");
  }
  return (c_x - p_x) / (c_time - p_time);  // TODO refine this(use low pass filter?)
}

double ScenarioAPIAutoware::getJerk(
  const std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr,
  const std::shared_ptr<geometry_msgs::TwistStamped> previous_twist_ptr,
  const std::shared_ptr<geometry_msgs::TwistStamped> second_previous_twist_ptr)
{
  double c_a = getAccel(current_twist_ptr, previous_twist_ptr);
  double p_a = getAccel(previous_twist_ptr, second_previous_twist_ptr);
  double c_time =
    (current_twist_ptr->header.stamp.toSec() + previous_twist_ptr->header.stamp.toSec()) / 2.0;
  double p_time =
    (previous_twist_ptr->header.stamp.toSec() + second_previous_twist_ptr->header.stamp.toSec()) /
    2.0;
  if (c_time - p_time <= 0) {
    ROS_WARN("return invalid jerk because of invalid timestamp(twist_ptr)");
  }
  return (c_a - p_a) / (c_time - p_time);
}

void ScenarioAPIAutoware::updateTotalMoveDistance()
{
  //calculate valid velocity
  //max... max_velocity + margin
  const static double valid_max_velocity_thresh_ =
    autoware_max_velocity_ + valid_max_velocity_margin_;
  //min... 3 sigma of noise
  const static double valid_min_velocity_thresh_ =
    add_simulator_noise_ * (3 * std::sqrt(2) * simulator_noise_pos_dev_) / slow_time_control_dt_;

  if (!current_pose_ptr_) {
    return;
  }

  static auto previous_pose_ptr_ = *current_pose_ptr_;

  //calculate delta-distance
  const double dt =
    current_pose_ptr_->header.stamp.toSec() - previous_pose_ptr_.header.stamp.toSec();
  const double dx = current_pose_ptr_->pose.position.x - previous_pose_ptr_.pose.position.x;
  const double dy = current_pose_ptr_->pose.position.y - previous_pose_ptr_.pose.position.y;
  const double dist = std::hypot(dx, dy);  //do not consider height
  const double v = dist / dt;

  // check invalid move
  if (std::isnan(v)) {
    previous_pose_ptr_ = *current_pose_ptr_;
    return;
  }
  if (std::fabs(v) > valid_max_velocity_thresh_) {
    ROS_ERROR_STREAM(
      "Detect invalid movement. Do not add delta-pose to total-move-distance. v=( " << v << " )");
    previous_pose_ptr_ = *current_pose_ptr_;
    return;
  }

  //avoid to add distance by simulator-noise to total move distance
  if (std::fabs(v) < valid_min_velocity_thresh_) {
    previous_pose_ptr_ = *current_pose_ptr_;
    return;
  }

  total_move_distance_ += dist;
  previous_pose_ptr_ = *current_pose_ptr_;
}

bool ScenarioAPIAutoware::shiftEgoPose(
  const geometry_msgs::Pose & pose, const std::string frame_type, geometry_msgs::Pose * shift_pose)
{
  //shift pose from farame_type to "Center"

  if (frame_type == "Center") {
    //no shift
    *shift_pose = pose;
    return true;
  }

  if (frame_type == "Front") {
    //shift to back
    *shift_pose = movePose(pose, -(vehicle_data_.wheel_base + vehicle_data_.front_overhang));
    return true;
  }

  if (frame_type == "Rear") {
    //shift to front
    *shift_pose = movePose(pose, vehicle_data_.rear_overhang);
    return true;
  }

  ROS_ERROR_STREAM(
    "shiftEGoPose supports only base_link, front_link, and rear_link as frame_type. "
    << "Now, frame_type is " << frame_type << ".");
  return false;
}

// additonal self vehicle API
bool ScenarioAPIAutoware::willLaneChange()
{
  ROS_WARN_DELAYED_THROTTLE(5.0, "willLaneChange is not perfectly implemented yet.");
  // TODO fix this (Now, this returns true when left/right turn)
  return (getLeftBlinker() || getRightBlinker());
}

bool ScenarioAPIAutoware::getLeftBlinker(
  std::shared_ptr<autoware_vehicle_msgs::TurnSignal> turn_signal_ptr)
{
  return (turn_signal_ptr->data == autoware_vehicle_msgs::TurnSignal::LEFT);
};
bool ScenarioAPIAutoware::getRightBlinker(
  std::shared_ptr<autoware_vehicle_msgs::TurnSignal> turn_signal_ptr)
{
  return (turn_signal_ptr->data == autoware_vehicle_msgs::TurnSignal::RIGHT);
}

bool ScenarioAPIAutoware::getLeftBlinker() { return getLeftBlinker(turn_signal_ptr_); }

bool ScenarioAPIAutoware::getRightBlinker() { return getRightBlinker(turn_signal_ptr_); }

bool ScenarioAPIAutoware::approveLaneChange(bool approve_lane_change)
{
  std_msgs::Bool boolmsg;
  boolmsg.data = approve_lane_change;
  pub_lane_change_permission_.publish(boolmsg);
  return true;  // TODO check successs
}

// sensor API
std::shared_ptr<sensor_msgs::PointCloud2> ScenarioAPIAutoware::getPointCloud() { return pcl_ptr_; }

// lane API
bool ScenarioAPIAutoware::getCurrentLaneID(int & current_id, double max_dist, double max_delta_yaw)
{
  return getCurrentLaneID(current_id, current_pose_ptr_, lanelet_map_ptr_, max_dist, max_delta_yaw);
}

bool ScenarioAPIAutoware::getCurrentLaneID(
  int & current_id, const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr, double max_dist,
  double max_delta_yaw)
{
  lanelet::BasicPoint2d search_point(current_pose->pose.position.x, current_pose->pose.position.y);
  const auto nearest_lanelets = lanelet::geometry::findNearest(
    lanelet_map_ptr->laneletLayer, search_point, 10);  // distance, lanelet
  lanelet::Lanelet target_closest_lanelet;
  bool is_found_target_closest_lanelet = false;
  double min_dist = max_dist;
  for (const auto & lanelet : nearest_lanelets) {
    double current_yaw = tf2::getYaw(current_pose->pose.orientation);
    double lane_yaw = lanelet::utils::getLaneletAngle(lanelet.second, current_pose->pose.position);
    double delta_yaw = std::abs(normalizeRadian(current_yaw - lane_yaw));
    if (lanelet.first < max_dist && delta_yaw < max_delta_yaw and lanelet.first < min_dist) {
      min_dist = lanelet.first;
      target_closest_lanelet = lanelet.second;
      is_found_target_closest_lanelet = true;
    }
  }

  if (is_found_target_closest_lanelet) {
    closest_lanelet_ptr_ = std::make_shared<lanelet::Lanelet>(target_closest_lanelet);
    current_id = (int)target_closest_lanelet.id();
    return true;
  } else {
    // closest_lanelet_ptr_.reset();
    return false;
  }
}

bool ScenarioAPIAutoware::getCurrentLeftLaneID(
  int & current_left_id, const std::shared_ptr<lanelet::Lanelet> current_lane)
{
  if (current_lane == nullptr) {
    ROS_WARN("cannot get left lane id (current_lane is nullptr)");
    return false;
  }

  auto left_lane = routing_graph_ptr_->left(*current_lane);
  if (!left_lane) {
    // current lane has not left lane
    return false;
  }
  current_left_id = left_lane->id();
  return true;
}

bool ScenarioAPIAutoware::isChangeLaneID()
{
  ROS_WARN("isChangeLaneID is not implemented yet.");
  // TODO
  return false;
}

bool ScenarioAPIAutoware::getDistancefromCenterLine(double & dist_from_center_line)
{
  return getDistancefromCenterLine(dist_from_center_line, current_pose_ptr_, closest_lanelet_ptr_);
}

bool ScenarioAPIAutoware::getDistancefromCenterLine(
  double & dist_from_center, const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr, lanelet::Id lane_id)
{
  std::shared_ptr<lanelet::Lanelet> lanelet =
    std::make_shared<lanelet::Lanelet>(lanelet_map_ptr->laneletLayer.get(lane_id));
  return getDistancefromCenterLine(dist_from_center, current_pose, lanelet);
}

bool ScenarioAPIAutoware::getDistancefromCenterLine(
  double & dist_from_center, const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
  std::shared_ptr<lanelet::Lanelet> current_lanelet)
{
  if (current_lanelet == nullptr or current_pose == nullptr) {
    ROS_WARN_DELAYED_THROTTLE(5.0, "cannot get distance from centerline (nullptr)");
    return false;
  }

  const auto centerline = current_lanelet->centerline2d();
  if (centerline.empty()) {
    ROS_WARN_DELAYED_THROTTLE(5.0, "cannot get distance from centerline (invalid centerline)");
    return false;
  }

  double dist_from_center_line = 100.0;  // TODO -> fix this
  for (size_t i = 0; i < centerline.size() - 1; i++) {
    const Line laneline = {{centerline[i].x(), centerline[i].y()},
                           {centerline[i + 1].x(), centerline[i + 1].y()}};
    const Point point2d(current_pose->pose.position.x, current_pose->pose.position.y);
    double dist = bg::distance(laneline, point2d);
    if (dist <= dist_from_center_line) dist_from_center_line = dist;
  }
  dist_from_center = dist_from_center_line;
  return true;
}

bool ScenarioAPIAutoware::isInLane(
  const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
  std::shared_ptr<lanelet::Lanelet> current_lanelet)
{
  const auto poly = current_lanelet->polygon2d().basicPolygon();
  const Point point2d(current_pose->pose.position.x, current_pose->pose.position.y);
  return (bg::distance(poly, point2d) <= 0);
}

bool ScenarioAPIAutoware::isInLane() { return isInLane(current_pose_ptr_, closest_lanelet_ptr_); }

// traffic light API

bool ScenarioAPIAutoware::setTrafficLightColor(
  const int traffic_id, const std::string & traffic_color)
{
  for (auto & tl_state : traffic_light_state_.states) {
    if (tl_state.id == traffic_id) {
      // modify existed traffic light state
      // TODO probabilistic expression of traffic light
      for (auto & lamp : tl_state.lamp_states) {
        if (
          lamp.type == autoware_perception_msgs::LampState::RED ||
          lamp.type == autoware_perception_msgs::LampState::YELLOW ||
          lamp.type == autoware_perception_msgs::LampState::GREEN) {
          //if lamp color is already given, change color
          lamp.confidence = 1.0;
          lamp.type = getTrafficLampStateFromString(traffic_color);
          return true;
        }
      }
      //if lamp color is not given yet, append color
      autoware_perception_msgs::LampState color_lamp;
      color_lamp.confidence = 1.0;
      color_lamp.type = getTrafficLampStateFromString(traffic_color);
      tl_state.lamp_states.emplace_back(color_lamp);
      return true;
    }
  }

  // if traffic id does not exist, add new traffic light state
  autoware_perception_msgs::TrafficLightState tl_state;
  tl_state.id = traffic_id;
  autoware_perception_msgs::LampState lmp_state;
  lmp_state.confidence = 1.0;
  lmp_state.type = getTrafficLampStateFromString(traffic_color);
  tl_state.lamp_states.emplace_back(lmp_state);
  traffic_light_state_.states.emplace_back(tl_state);

  return true;
}

bool ScenarioAPIAutoware::setTrafficLightsColor(
  const int traffic_relation_id, const std::string & traffic_color)
{
  lanelet::LineStringsOrPolygons3d traffic_lights;
  if (!getTrafficLights(traffic_relation_id, traffic_lights)) {
    return false;
  }

  for (const auto traffic_light : traffic_lights) {
    if (!setTrafficLightColor(traffic_light.id(), traffic_color)) {
      return false;
    }
  }
  return true;
}

bool ScenarioAPIAutoware::setTrafficLightArrow(
  const int traffic_id, const std::string & traffic_arrow)
{
  for (auto & tl_state : traffic_light_state_.states) {
    if (tl_state.id == traffic_id) {
      // modify existed traffic light state
      // TODO probabilistic expression of traffic light
      for (auto lamp : tl_state.lamp_states) {
        if (lamp.type == getTrafficLampStateFromString(traffic_arrow)) {
          //if same arrow is already given, do nothing
          return true;
        }
      }
      //if same arrow is not given yet, append color
      autoware_perception_msgs::LampState arrow_lamp;
      arrow_lamp.confidence = 1.0;
      arrow_lamp.type = getTrafficLampStateFromString(traffic_arrow);
      tl_state.lamp_states.emplace_back(arrow_lamp);
      return true;
    }
  }

  // if traffic id does not exist, add new traffic light state
  autoware_perception_msgs::TrafficLightState tl_state;
  tl_state.id = traffic_id;
  autoware_perception_msgs::LampState lmp_state;
  lmp_state.confidence = 1.0;
  lmp_state.type = getTrafficLampStateFromString(traffic_arrow);
  tl_state.lamp_states.emplace_back(lmp_state);
  traffic_light_state_.states.emplace_back(tl_state);

  return true;
}

bool ScenarioAPIAutoware::setTrafficLightsArrow(
  const int traffic_relation_id, const std::string & traffic_arrow)
{
  lanelet::LineStringsOrPolygons3d traffic_lights;
  if (!getTrafficLights(traffic_relation_id, traffic_lights)) {
    return false;
  }

  for (const auto traffic_light : traffic_lights) {
    if (!setTrafficLightArrow(traffic_light.id(), traffic_arrow)) {
      return false;
    }
  }
  return true;
}

bool ScenarioAPIAutoware::resetTrafficLightColor(const int traffic_id)
{
  for (auto & tl_state : traffic_light_state_.states) {
    if (tl_state.id == traffic_id) {
      // modify existed traffic light state
      // TODO probabilistic expression of traffic light
      for (int i = 0; i < tl_state.lamp_states.size(); i++) {
        if (
          //remove color
          tl_state.lamp_states[i].type == autoware_perception_msgs::LampState::RED ||
          tl_state.lamp_states[i].type == autoware_perception_msgs::LampState::YELLOW ||
          tl_state.lamp_states[i].type == autoware_perception_msgs::LampState::GREEN) {
          tl_state.lamp_states.erase(tl_state.lamp_states.begin() + i);
          return true;
        }
      }
      //if there are no color, do nothing
      return true;
    }
  }
  //do nothing(invalid id is allowed)
  return true;
}

bool ScenarioAPIAutoware::resetTrafficLightsColor(const int traffic_relation_id)
{
  {
    lanelet::LineStringsOrPolygons3d traffic_lights;
    if (!getTrafficLights(traffic_relation_id, traffic_lights)) {
      return false;
    }

    for (const auto traffic_light : traffic_lights) {
      if (!resetTrafficLightColor(traffic_light.id())) {
        return false;
      }
    }
    return true;
  }
}

bool ScenarioAPIAutoware::resetTrafficLightArrow(const int traffic_id)
{
  for (auto & tl_state : traffic_light_state_.states) {
    if (tl_state.id == traffic_id) {
      // modify existed traffic light state
      // TODO probabilistic expression of traffic light
      for (auto & lamp : tl_state.lamp_states) {
        if (
          //remove lamp_state without color
          lamp.type == autoware_perception_msgs::LampState::RED ||
          lamp.type == autoware_perception_msgs::LampState::YELLOW ||
          lamp.type == autoware_perception_msgs::LampState::GREEN) {
          const uint32_t color_type = lamp.type;
          //reset lamp_states and push color_type
          tl_state.lamp_states.clear();
          autoware_perception_msgs::LampState lamp;
          lamp.type = color_type;
          lamp.confidence = 1.0;
          tl_state.lamp_states.emplace_back(lamp);
          return true;
        }
      }
      //if there are no color, clear all lamp states
      tl_state.lamp_states.clear();
      return true;
    }
  }
  //do nothing(invalid id is allowed)
  return true;
}

bool ScenarioAPIAutoware::resetTrafficLightsArrow(const int traffic_relation_id)
{
  {
    lanelet::LineStringsOrPolygons3d traffic_lights;
    if (!getTrafficLights(traffic_relation_id, traffic_lights)) {
      return false;
    }

    for (const auto traffic_light : traffic_lights) {
      if (!resetTrafficLightArrow(traffic_light.id())) {
        return false;
      }
    }
    return true;
  }
}

bool ScenarioAPIAutoware::getTrafficLightColor(
  const int traffic_relation_id, std::string * traffic_color)
{
  lanelet::LineStringsOrPolygons3d traffic_lights;
  if (!getTrafficLights(traffic_relation_id, traffic_lights)) {
    ROS_WARN_STREAM(
      "traffic light id:" << traffic_relation_id << " is invalid. cannot get traffic light");
    return false;
  }
  for (const auto & traffic_light : traffic_lights) {
    for (const auto & tl_state : traffic_light_state_.states) {
      if (traffic_light.id() == tl_state.id) {
        for (const auto & lamp_state : tl_state.lamp_states) {
          const auto lamp = static_cast<uint8_t>(lamp_state.type);
          if (
            lamp == autoware_perception_msgs::LampState::RED ||
            lamp == autoware_perception_msgs::LampState::YELLOW ||
            lamp == autoware_perception_msgs::LampState::GREEN) {
            *traffic_color = getTrafficLampStringFromState(lamp);
            return true;
          }
        }
      }
    }
    //no lamp state
    *traffic_color = "";
    return true;
  }
  ROS_WARN_STREAM("traffic light id:" << traffic_relation_id << "is not registered");
  return false;
}

bool ScenarioAPIAutoware::getTrafficLightArrow(
  const int traffic_relation_id, std::vector<std::string> * const traffic_arrow)
{
  lanelet::LineStringsOrPolygons3d traffic_lights;
  if (!getTrafficLights(traffic_relation_id, traffic_lights)) {
    ROS_WARN_STREAM(
      "traffic light id:" << traffic_relation_id << " is invalid. cannot get traffic light");
    return false;
  }
  traffic_arrow->clear();
  for (const auto traffic_light : traffic_lights) {
    for (const auto tl_state : traffic_light_state_.states) {
      if (traffic_light.id() == tl_state.id) {
        for (const auto & lamp_state : tl_state.lamp_states) {
          const auto lamp = static_cast<uint8_t>(lamp_state.type);
          if (
            lamp == autoware_perception_msgs::LampState::UP ||
            lamp == autoware_perception_msgs::LampState::DOWN ||
            lamp == autoware_perception_msgs::LampState::LEFT ||
            lamp == autoware_perception_msgs::LampState::RIGHT) {
            traffic_arrow->emplace_back(getTrafficLampStringFromState(lamp));
          }
        }
        return true;
      }
    }
  }
  ROS_WARN_STREAM("color of traffic light id:" << traffic_relation_id << "is not registered");
  return false;
}

std::vector<geometry_msgs::Point> ScenarioAPIAutoware::getTrafficLightPosition(
  const int traffic_relation_id)
{
  std::vector<geometry_msgs::Point> traffic_points;
  lanelet::LineStringsOrPolygons3d traffic_lights;
  if (!getTrafficLights(traffic_relation_id, traffic_lights)) {
    return traffic_points;
  }
  for (const auto traffic_light : traffic_lights) {
    auto tl = traffic_light.lineString();
    geometry_msgs::Point tl_point;
    tl_point.x = tl->front().x();
    tl_point.y = tl->front().y();
    tl_point.z = tl->front().z();
    traffic_points.emplace_back(tl_point);
  }
  return traffic_points;
}

bool ScenarioAPIAutoware::getTrafficLineCenterPosition(
  const int traffic_relation_id, geometry_msgs::Point & line_center)
{
  if (!lanelet_map_ptr_->regulatoryElementLayer.exists(traffic_relation_id)) {
    ROS_WARN_STREAM(
      "RegulatoryElement, id:" << traffic_relation_id << "does not exist. Check the traffic id");
    return false;
  }

  auto traffic_element = lanelet_map_ptr_->regulatoryElementLayer.get(traffic_relation_id);

  auto traffic_light_reg_elem = std::dynamic_pointer_cast<lanelet::TrafficLight>(traffic_element);
  if (!traffic_light_reg_elem) {
    ROS_WARN_STREAM(
      "Result of dynamic pointer cast of regulatoryElement, id:"
      << traffic_relation_id << "is nullptr. Check the traffic id");
    return false;
  }

  lanelet::ConstLineString3d stop_line = *(traffic_light_reg_elem->stopLine());
  const int sl_size = stop_line.size();
  if (sl_size == 0) {
    ROS_WARN_STREAM(
      "Stop line of traffic_relation id:" << traffic_relation_id << " does not exist");
    return false;
  }

  if (sl_size % 2 == 0) {
    line_center.x = (stop_line[sl_size / 2].x() + stop_line[sl_size / 2 - 1].x()) / 2.0;
    line_center.y = (stop_line[sl_size / 2].y() + stop_line[sl_size / 2 - 1].y()) / 2.0;
    line_center.z = (stop_line[sl_size / 2].z() + stop_line[sl_size / 2 - 1].z()) / 2.0;
  } else {
    line_center.x = stop_line[(sl_size - 1) / 2].x();
    line_center.y = stop_line[(sl_size - 1) / 2].y();
    line_center.z = stop_line[(sl_size - 1) / 2].z();
  }
  return true;
}

bool ScenarioAPIAutoware::getTrafficLineCenterPose(
  const int traffic_relation_id, geometry_msgs::Pose & line_pose)
{
  //get stop line position
  geometry_msgs::Pose stop_line_center;
  if (!getTrafficLineCenterPosition(traffic_relation_id, stop_line_center.position)) {
    return false;
  }

  lanelet::BasicPoint2d search_point(stop_line_center.position.x, stop_line_center.position.y);
  const auto nearest_lanelet =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 1);

  //get stop line orientation(input orientation of nearest lane)
  if (nearest_lanelet.empty()) {
    ROS_WARN_STREAM("Failed to find the closest lane of stop line");
    return false;
  }

  double yaw =
    lanelet::utils::getLaneletAngle(nearest_lanelet.front().second, stop_line_center.position);
  stop_line_center.orientation = quatFromYaw(yaw);

  line_pose = stop_line_center;
  return true;
}

bool ScenarioAPIAutoware::getDistanceToTrafficLight(
  const int traffic_relation_id, const geometry_msgs::Pose self_pose, double & distance)
{
  const auto traffic_lights = getTrafficLightPosition(traffic_relation_id);
  double min_distance = 1e5;
  bool exist_traffic_light = false;
  for (const auto traffic_light : traffic_lights) {
    double dx = self_pose.position.x - traffic_light.x;
    double dy = self_pose.position.y - traffic_light.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (min_distance > dist) {
      min_distance = dist;
      exist_traffic_light = true;
    }
  }

  if (!exist_traffic_light) {
    return false;
  }

  distance = min_distance;
  return true;
}

bool ScenarioAPIAutoware::getDistanceToTrafficLine(
  const int traffic_relation_id, const geometry_msgs::Pose self_pose, double & distance)
{
  geometry_msgs::Point line_center;
  if (!getTrafficLineCenterPosition(traffic_relation_id, line_center)) {
    return false;
  }
  double dx = self_pose.position.x - line_center.x;
  double dy = self_pose.position.y - line_center.y;
  distance = std::sqrt(dx * dx + dy * dy);
  return true;
}

bool ScenarioAPIAutoware::checkOverTrafficLine(
  const int traffic_relation_id, const geometry_msgs::Pose self_pose, bool & over_line,
  double judge_dist_thresh_)
{
  double distance2line;
  if (!getDistanceToTrafficLine(traffic_relation_id, self_pose, distance2line)) {
    return false;
  }

  if (distance2line > judge_dist_thresh_) {
    //when traffic light is too far, over_line is judged as false.
    over_line = false;
    return true;
  }

  geometry_msgs::Pose line_center;
  if (!getTrafficLineCenterPose(traffic_relation_id, line_center)) {
    return false;
  }

  const double dx = self_pose.position.x - line_center.position.x;
  const double dy = self_pose.position.y - line_center.position.y;
  const double lane2self_yaw = std::atan2(dy, dx);
  const double lane_yaw = yawFromQuat(line_center.orientation);

  if (std::fabs(normalizeRadian(lane_yaw - lane2self_yaw)) <= M_PI / 2.0) {
    //base link is over the line
    over_line = true;
    return true;
  }

  if (distance2line < vehicle_data_.front_from_base()) {
    //front of the car is over the line
    over_line = true;
    return true;
  }

  return true;
}

bool ScenarioAPIAutoware::getTrafficLights(
  const int traffic_relation_id, lanelet::LineStringsOrPolygons3d & traffic_lights)
{
  if (!lanelet_map_ptr_->regulatoryElementLayer.exists(traffic_relation_id)) {
    ROS_WARN_STREAM(
      "regulatoryElement, id:" << traffic_relation_id << "does not exist. Check the traffic id");
    return false;
  }
  auto traffic_element = lanelet_map_ptr_->regulatoryElementLayer.get(traffic_relation_id);

  auto traffic_light_reg_elem = std::dynamic_pointer_cast<lanelet::TrafficLight>(traffic_element);
  if (!traffic_light_reg_elem) {
    ROS_WARN_STREAM(
      "Result of dynamic pointer cast of regulatoryElement, id:"
      << traffic_relation_id << "is nullptr. Check the traffic id");
    return false;
  }

  traffic_lights = traffic_light_reg_elem->trafficLights();
  return true;
}

uint8_t ScenarioAPIAutoware::getTrafficLampStateFromString(const std::string & traffic_state)
{
  std::string traffic_state_sc = traffic_state;
  transform(traffic_state_sc.begin(), traffic_state_sc.end(), traffic_state_sc.begin(), toupper);

  if (traffic_state_sc == "RED") {
    return autoware_perception_msgs::LampState::RED;
  } else if (traffic_state_sc == "YELLOW") {
    return autoware_perception_msgs::LampState::YELLOW;
  } else if ((traffic_state_sc == "GREEN" || traffic_state_sc == "BLUE")) {
    return autoware_perception_msgs::LampState::GREEN;
  } else if (traffic_state_sc == "UP" || traffic_state_sc == "STRAIGHT") {
    return autoware_perception_msgs::LampState::UP;
  } else if (traffic_state_sc == "DOWN") {
    //??? what is "DOWN" ...?
    return autoware_perception_msgs::LampState::DOWN;
  } else if (traffic_state_sc == "LEFT") {
    return autoware_perception_msgs::LampState::LEFT;
  } else if (traffic_state_sc == "RIGHT") {
    return autoware_perception_msgs::LampState::RIGHT;
  }
  ROS_ERROR_STREAM("invalid traffic_state in getTrafficLampStateFromString: " << traffic_state_sc);
  return 0;
}

std::string ScenarioAPIAutoware::getTrafficLampStringFromState(const uint8_t lamp_state)
{
  //reference: https://github.com/tier4/ScenarioFormat/blob/master/format/definition.md/#ArrowType
  if (lamp_state == autoware_perception_msgs::LampState::RED) {
    return "Red";
  }
  if (lamp_state == autoware_perception_msgs::LampState::YELLOW) {
    return "Yellow";
  }
  if (lamp_state == autoware_perception_msgs::LampState::GREEN) {
    return "Green";
  }
  if (lamp_state == autoware_perception_msgs::LampState::UP) {
    return "Straight";
  }
  if (lamp_state == autoware_perception_msgs::LampState::DOWN) {
    return "Down";
  }
  if (lamp_state == autoware_perception_msgs::LampState::LEFT) {
    return "Left";
  }
  if (lamp_state == autoware_perception_msgs::LampState::RIGHT) {
    return "Right";
  }
  ROS_ERROR_STREAM("invalid lamp state in getTrafficLampStringFromState");
  return "";
}

void ScenarioAPIAutoware::pubTrafficLight()
{
  traffic_light_state_.header.frame_id = camera_frame_id_;
  traffic_light_state_.header.stamp = ros::Time::now();
  pub_traffic_detection_result_.publish(traffic_light_state_);
}

// util
Polygon ScenarioAPIAutoware::getSelfPolygon2D(Vehicle_Data vd)
{
  double left = vd.left_from_base();
  double right = vd.right_from_base();
  double front = vd.front_from_base();
  double rear = vd.rear_from_base();
  double top = vd.top_from_base();
  double bottom = vd.bottom_from_base();

  // create polygon of self body shape
  Polygon poly;
  bg::exterior_ring(poly) =
    boost::assign::list_of<Point>(front, left)(front, right)(rear, right)(rear, left)(front, left);
  return poly;
}
