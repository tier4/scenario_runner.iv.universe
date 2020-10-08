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

#ifndef SCENARIO_API_SCENARIO_API_AUTOWARE_H_INCLUDED
#define SCENARIO_API_SCENARIO_API_AUTOWARE_H_INCLUDED

#include <autoware_perception_msgs/Semantic.h>
#include <autoware_perception_msgs/Shape.h>
#include <autoware_perception_msgs/TrafficLightStateArray.h>
#include <autoware_planning_msgs/Route.h>
#include <autoware_system_msgs/AutowareState.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <scenario_api_utils/scenario_api_utils.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <boost/uuid/uuid_generators.hpp>
#include <chrono>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/* define vehicle shape structure*/
struct Vehicle_Data
{
  double wheel_radius;
  double wheel_width;
  double wheel_base;
  double wheel_tread;
  double front_overhang;
  double rear_overhang;
  double vehicle_height;
  double front_from_base() { return front_overhang + wheel_base; };
  double rear_from_base() { return -rear_overhang; };
  double left_from_base() { return wheel_tread / 2.0 + wheel_width / 2.0; };
  double right_from_base() { return -(wheel_tread / 2.0 + wheel_width / 2.0); };
  double top_from_base() { return vehicle_height; };
  double bottom_from_base() { return 0.0; };
};

class ScenarioAPIAutoware
{
public:
  /**
   * @brief constructor
   */
  ScenarioAPIAutoware();

  /**
   * @brief destructor
   */
  ~ScenarioAPIAutoware();

  // basic API
  bool isAPIReady();    // check ready for all callback function
  bool waitAPIReady();  // wait until ready

  // start API
  bool sendStartPoint(
    const geometry_msgs::Pose pose, const bool wait_autoware_status,
    const std::string & frame_type);
  bool sendGoalPoint(
    const geometry_msgs::Pose pose, const bool wait_autoware_status,
    const std::string & frame_type);
  bool sendCheckPoint(
    const geometry_msgs::Pose pose, const bool wait_autoware_status,
    const std::string & frame_type);
  bool sendStartVelocity(const double velocity);
  bool sendEngage(const bool engage);
  bool waitAutowareInitialize();
  bool isAutowareReadyInitialize();
  bool isAutowareReadyRouting();
  bool setMaxSpeed(double velocity);

  // basic self vehicle API
  Pose2D getCurrentPose();
  geometry_msgs::PoseStamped getCurrentPoseRos();
  Polygon getSelfPolygon2D();
  double getVehicleTopFromBase();
  double getVehicleBottomFromBase();
  double getVelocity();
  double getAccel();
  double getJerk();
  double getMoveDistance();

  // basic self vehicle API (tools)
  bool shiftEgoPose(
    const geometry_msgs::Pose & pose, const std::string frame_type,
    geometry_msgs::Pose * shift_pose);

  // additonal self vehicle API
  bool willLaneChange();  // TODO //!< @brief t4b try to change lanes or not
  bool getLeftBlinker();
  bool getRightBlinker();
  bool approveLaneChange(bool approve_lane_change);

  // sensor API
  std::shared_ptr<sensor_msgs::PointCloud2> getPointCloud();

  // lane API
  bool getCurrentLaneID(
    int & current_id, double max_dist = 3.0, double max_deleta_yaw = M_PI / 4.0);
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
  bool getTrafficLineCenterPose(const int traffic_relation_id, geometry_msgs::Pose & line_pose);
  bool getDistanceToTrafficLight(
    const int traffic_relation_id, const geometry_msgs::Pose self_pose, double & distance);
  bool getDistanceToTrafficLine(
    const int traffic_relation_id, const geometry_msgs::Pose self_pose, double & distance);
  bool checkOverTrafficLine(
    const int traffic_relation_id, const geometry_msgs::Pose self_pose, bool & over_line,
    double judge_dist_thresh = 30.0);

private:
  ros::NodeHandle nh_;         //!< @brief ros node handle
  ros::NodeHandle pnh_;        //!< @brief private ros node handle
  ros::Subscriber sub_state_;  //!< @brief topic subscriber for autoware state
  ros::Subscriber sub_pcl_;    //!< @brief topic subscriber for pcl
  ros::Subscriber sub_map_;    //!< @brief topic subscriber for map
  ros::Subscriber
    sub_route_;  //!< @brief topic subscriber for current route (for check of autoware ready)
  ros::Subscriber sub_twist_;           //!< @brief topic subscriber for twist
  ros::Subscriber sub_turn_signal_;     //!< @brief topic subscriber for turn signal(blinker)
  ros::Timer timer_control_fast_;       //!< @brief timer for getting self-position etc
  ros::Timer timer_control_slow_;       //!< @brief timer for getting total-move distance etc
  ros::Publisher pub_start_point_;      //!< @brief topic pubscriber for start point
  ros::Publisher pub_goal_point_;       //!< @brief topic pubscriber for goal point
  ros::Publisher pub_check_point_;      //!< @brief topic pubscriber for check point
  ros::Publisher pub_start_velocity_;   //!< @brief topic @publisher for initial velocity
  ros::Publisher pub_max_velocity_;     //!< @brief topic pubscriber for max velocity
  ros::Publisher pub_autoware_engage_;  //!< @brief topic pubscriber for autoware engage
  ros::Publisher
    pub_traffic_detection_result_;  //!< @brief topic pubscriber for traffic detection result
  ros::Publisher
    pub_lane_change_permission_;  //!< @brief topic pubscriber for approval of lane change

  const double fast_time_control_dt_ = 0.01;
  const double slow_time_control_dt_ = 0.2;

  bool is_autoware_ready_initialize;
  bool is_autoware_ready_routing;
  std::string autoware_state_;
  autoware_perception_msgs::TrafficLightStateArray traffic_light_state_;
  double total_move_distance_;

  // get msg from topic
  std::shared_ptr<sensor_msgs::PointCloud2> pcl_ptr_;
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;
  std::shared_ptr<geometry_msgs::PoseStamped> previous_pose_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> previous_twist_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> second_previous_twist_ptr_;
  std::shared_ptr<autoware_vehicle_msgs::TurnSignal> turn_signal_ptr_;
  Vehicle_Data vehicle_data_;

  // lanelet
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  std::shared_ptr<lanelet::Lanelet> closest_lanelet_ptr_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  // Traffic Light
  std::string camera_frame_id_;

  //parameter for getMoveDistance
  const double valid_max_velocity_margin_ = 5.0;  //[m/s], 18kmph
  bool add_simulator_noise_;
  double simulator_noise_pos_dev_;
  double autoware_max_velocity_;

  // callback function
  void getCurrentPoseFromTF();
  void timerCallbackFast(const ros::TimerEvent &);
  void timerCallbackSlow(const ros::TimerEvent &);
  void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr & msg);
  void callbackMap(const autoware_lanelet2_msgs::MapBin & msg);
  void callbackRoute(const autoware_planning_msgs::Route & msg);
  void callbackStatus(const autoware_system_msgs::AutowareState & msg);
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);
  void callbackTurnSignal(const autoware_vehicle_msgs::TurnSignal::ConstPtr & msg);

  // function for start API
  bool checkState(const std::string state);
  bool waitState(const std::string state);

  // function for basic self vehicle API
  double getAccel(
    const std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr,
    const std::shared_ptr<geometry_msgs::TwistStamped> previous_twist_ptr);

  double getJerk(
    const std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr,
    const std::shared_ptr<geometry_msgs::TwistStamped> previous_twist_ptr,
    const std::shared_ptr<geometry_msgs::TwistStamped> second_previous_twist_ptr);
  void updateTotalMoveDistance();

  // function for additonal self vehicle API
  bool getLeftBlinker(std::shared_ptr<autoware_vehicle_msgs::TurnSignal> turn_signal_ptr);
  bool getRightBlinker(std::shared_ptr<autoware_vehicle_msgs::TurnSignal> turn_signal_ptr);

  // function for lane API
  bool getCurrentLaneID(
    int & current_id, const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
    const lanelet::LaneletMapPtr & lanelet_map_ptr, double max_dist, double max_deleta_yaw);
  bool getCurrentLeftLaneID(
    int & current_left_id, const std::shared_ptr<lanelet::Lanelet> current_lane);
  bool getDistancefromCenterLine(
    double & dist_from_center, const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
    std::shared_ptr<lanelet::Lanelet> current_lanelet);
  bool getDistancefromCenterLine(
    double & dist_from_center, const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
    const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr, lanelet::Id lane_id);

  bool isInLane(
    const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
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
  std::vector<geometry_msgs::Point> getTrafficLightPosition(const int traffic_relation_id);
  bool getTrafficLineCenterPosition(
    const int traffic_relation_id, geometry_msgs::Point & line_center);

  // others
  Polygon getSelfPolygon2D(Vehicle_Data vd);
};

#endif  // SCENARIO_API_SCENARIO_API_AUTOWARE_H_INCLUDED
