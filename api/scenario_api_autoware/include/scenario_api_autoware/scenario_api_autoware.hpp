// Copyright 2018-2019 Autoware Foundation
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

#ifndef SCENARIO_API_SCENARIO_API_AUTOWARE_H_INCLUDED
#define SCENARIO_API_SCENARIO_API_AUTOWARE_H_INCLUDED

#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "autoware_perception_msgs/msg/traffic_light_state_array.hpp"
#include "autoware_planning_msgs/msg/route.hpp"
#include "autoware_planning_msgs/msg/lane_change_command.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_planning_msgs/msg/velocity_limit.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/primitives/BasicRegulatoryElements.h"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "lanelet2_routing/RoutingGraph.h"
#include "rclcpp/rclcpp.hpp"
#include "scenario_api_utils/scenario_api_utils.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "boost/geometry.hpp"

/* define vehicle shape structure*/
struct VehicleData
{
  double wheel_radius;
  double wheel_width;
  double wheel_base;
  double wheel_tread;
  double front_overhang;
  double rear_overhang;
  double vehicle_height;
  double max_longitudinal_offset;
  double min_longitudinal_offset;
  double max_height_offset;
  double min_height_offset;
  double left_from_base() {return wheel_tread / 2.0 + wheel_width / 2.0;}
  double right_from_base() {return -(wheel_tread / 2.0 + wheel_width / 2.0);}
};

class ScenarioAPIAutoware
{
public:
  using Point = boost::geometry::model::d2::point_xy<double>;
  using Polygon = boost::geometry::model::polygon<Point>;
  using Line = boost::geometry::model::linestring<Point>;

  /**
   * @brief constructor
   */
  ScenarioAPIAutoware(rclcpp::Node::SharedPtr node);

  /**
   * @brief destructor
   */
  ~ScenarioAPIAutoware();

  // basic API
  bool isAPIReady();    // check ready for all callback function
  bool waitAPIReady();  // wait until ready

  // start API
  bool sendStartPoint(
    const geometry_msgs::msg::Pose pose, const bool wait_autoware_status,
    const std::string & frame_type);
  bool sendGoalPoint(
    const geometry_msgs::msg::Pose pose, const bool wait_autoware_status,
    const std::string & frame_type);
  bool sendCheckPoint(
    const geometry_msgs::msg::Pose pose, const bool wait_autoware_status,
    const std::string & frame_type);
  bool sendStartVelocity(const double velocity);
  bool sendEngage(const bool engage);
  bool waitAutowareInitialize();
  bool isAutowareReadyInitialize();
  bool isAutowareReadyRouting();
  bool setMaxSpeed(double velocity);

  // basic self vehicle API
  Pose2D getCurrentPose();
  geometry_msgs::msg::PoseStamped getCurrentPoseRos();
  Polygon getSelfPolygon2D();
  double getVehicleTopFromBase();
  double getVehicleBottomFromBase();
  double getVelocity();
  double getAccel();
  double getJerk();
  double getMoveDistance();

  // basic self vehicle API (tools)
  bool shiftEgoPose(
    const geometry_msgs::msg::Pose & pose, const std::string frame_type,
    geometry_msgs::msg::Pose * shift_pose);

  // additional self vehicle API
  bool willLaneChange();  // TODO //!< @brief t4b try to change lanes or not
  bool getLeftBlinker();
  bool getRightBlinker();
  bool approveLaneChange(bool approve_lane_change);

  // sensor API
  std::shared_ptr<sensor_msgs::msg::PointCloud2> getPointCloud();

  // lane API
  bool getCurrentLaneID(
    int & current_id, double max_dist = 3.0, double max_delta_yaw = M_PI / 4.0);
  bool isChangeLaneID();  // future work: // TODO
  bool getDistancefromCenterLine(double & dist_from_center_line);
  bool isInLane();

  // traffic light API
  /* use relation id which has the tag of regulatory_element type and "traffic_light" subtype */
  bool setTrafficLightsColor(const int traffic_relation_id, const std::string & traffic_color);
  bool setTrafficLightsArrow(const int traffic_relation_id, const std::string & traffic_arrow);
  bool resetTrafficLightsColor(const int traffic_relation_id);
  bool resetTrafficLightsArrow(const int traffic_relation_id);
  bool getTrafficLightColor(const int traffic_relation_id, std::string * traffic_color);
  bool getTrafficLightArrow(
    const int traffic_relation_id, std::vector<std::string> * const traffic_arrow);
  bool getTrafficLineCenterPose(
    const int traffic_relation_id,
    geometry_msgs::msg::Pose & line_pose);
  bool getDistanceToTrafficLight(
    const int traffic_relation_id, const geometry_msgs::msg::Pose self_pose, double & distance);
  bool getDistanceToTrafficLine(
    const int traffic_relation_id, const geometry_msgs::msg::Pose self_pose, double & distance);
  bool checkOverTrafficLine(
    const int traffic_relation_id, const geometry_msgs::msg::Pose self_pose, bool & over_line,
    double judge_dist_thresh = 30.0);

private:
  rclcpp::Node::SharedPtr node_;
  /* Subscribers */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    sub_pcl_;  //!< @brief topic subscriber for pcl
  rclcpp::Subscription<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr
    sub_map_;  //!< @brief topic subscriber for map
  rclcpp::Subscription<autoware_planning_msgs::msg::Route>::SharedPtr
    sub_route_;  //!< @brief topic subscriber for current route (for check of autoware ready)
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr
    sub_state_;  //!< @brief topic subscriber for autoware state
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    sub_twist_;  //!< @brief topic subscriber for twist
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr
    sub_turn_signal_;  //!< @brief topic subscriber for turn signal(blinker)

  /* Timers */
  rclcpp::TimerBase::SharedPtr timer_control_fast_;       //!< @brief timer for getting self-position etc
  rclcpp::TimerBase::SharedPtr timer_control_slow_;       //!< @brief timer for getting total-move distance etc

  /* Publishers */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pub_start_point_;  //!< @brief topic publisher for start point
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pub_goal_point_;  //!< @brief topic publisher for goal point
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    pub_start_velocity_;  //!< @brief topic @publisher for initial velocity
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr
    pub_autoware_engage_;  //!< @brief topic publisher for autoware engage
  rclcpp::Publisher<autoware_planning_msgs::msg::VelocityLimit>::SharedPtr
    pub_max_velocity_;  //!< @brief topic publisher for max velocity
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneChangeCommand>::SharedPtr
    pub_lane_change_permission_;  //!< @brief topic publisher for approval of lane change
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pub_check_point_;  //!< @brief topic publisher for check point
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightStateArray>::SharedPtr
    pub_traffic_detection_result_;  //!< @brief topic publisher for traffic detection result

  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pub_pose_with_covariance_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Get Vehicle dimensions
  vehicle_info_util::VehicleInfo vehicle_info_;

  const double fast_time_control_dt_ = 0.01;
  const double slow_time_control_dt_ = 0.2;

  bool is_autoware_ready_initialize;
  bool is_autoware_ready_routing;
  std::string autoware_state_;
  autoware_perception_msgs::msg::TrafficLightStateArray traffic_light_state_;
  double total_move_distance_;

  // get msg from topic
  std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_ptr_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> previous_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr_;
  std::shared_ptr<geometry_msgs::msg::TwistStamped> previous_twist_ptr_;
  std::shared_ptr<geometry_msgs::msg::TwistStamped> second_previous_twist_ptr_;
  std::shared_ptr<autoware_vehicle_msgs::msg::TurnSignal> turn_signal_ptr_;
  VehicleData vehicle_data_;

  // lanelet
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  std::shared_ptr<lanelet::Lanelet> closest_lanelet_ptr_;

  // Traffic Light
  std::string camera_frame_id_;

  //parameter for getMoveDistance
  const double valid_max_velocity_margin_ = 5.0;  //[m/s], 18kmph
  bool add_simulator_noise_;
  double simulator_noise_pos_dev_;
  double autoware_max_velocity_;

  // callback function
  void getCurrentPoseFromTF();
  void timerCallbackFast();
  void timerCallbackSlow();
  void callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void callbackMap(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg);
  void callbackRoute(const autoware_planning_msgs::msg::Route::ConstSharedPtr msg);
  void callbackStatus(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void callbackTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void callbackTurnSignal(const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);

  // function for start API
  bool checkState(const std::string state);
  bool waitState(const std::string state);

  // function for basic self vehicle API
  double getAccel(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr,
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> previous_twist_ptr);

  double getJerk(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr,
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> previous_twist_ptr,
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> second_previous_twist_ptr);
  void updateTotalMoveDistance();

  // function for additional self vehicle API
  bool getLeftBlinker(std::shared_ptr<autoware_vehicle_msgs::msg::TurnSignal> turn_signal_ptr);
  bool getRightBlinker(std::shared_ptr<autoware_vehicle_msgs::msg::TurnSignal> turn_signal_ptr);

  // function for lane API
  bool getCurrentLaneID(
    int & current_id, const std::shared_ptr<geometry_msgs::msg::PoseStamped> & current_pose,
    const lanelet::LaneletMapPtr & lanelet_map_ptr, double max_dist, double max_delta_yaw);
  bool getCurrentLeftLaneID(
    int & current_left_id, const std::shared_ptr<lanelet::Lanelet> current_lane);
  bool getDistancefromCenterLine(
    double & dist_from_center,
    const std::shared_ptr<geometry_msgs::msg::PoseStamped> & current_pose,
    std::shared_ptr<lanelet::Lanelet> current_lanelet);
  bool getDistancefromCenterLine(
    double & dist_from_center,
    const std::shared_ptr<geometry_msgs::msg::PoseStamped> & current_pose,
    const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr, lanelet::Id lane_id);

  bool isInLane(
    const std::shared_ptr<geometry_msgs::msg::PoseStamped> & current_pose,
    std::shared_ptr<lanelet::Lanelet> current_lanelet);
  double getRateInLane();  // TODO get percentage of body in lanelet polygon

  // function for Traffic Light API
  void pubTrafficLight();
  uint8_t getTrafficLampStateFromString(const std::string & traffic_state);
  std::string getTrafficLampStringFromState(const uint8_t lamp_state);
  bool getTrafficLights(
    const int traffic_relation_id, lanelet::LineStringsOrPolygons3d & traffic_lights);
  /* use way id which has the tag of "traffic_light" type */
  bool setTrafficLightColor(const int traffic_id, const std::string & traffic_color);
  bool setTrafficLightArrow(const int traffic_id, const std::string & traffic_arrow);
  bool resetTrafficLightColor(const int traffic_id);
  bool resetTrafficLightArrow(const int traffic_id);
  std::vector<geometry_msgs::msg::Point> getTrafficLightPosition(const int traffic_relation_id);
  bool getTrafficLineCenterPosition(
    const int traffic_relation_id, geometry_msgs::msg::Point & line_center);

  // others
  Polygon getSelfPolygon2D(VehicleData vd);
};

#endif  // SCENARIO_API_SCENARIO_API_AUTOWARE_H_INCLUDED
