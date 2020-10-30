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
#ifndef SCENARIO_API_SCENARIO_API_CORE_H_INCLUDED
#define SCENARIO_API_SCENARIO_API_CORE_H_INCLUDED

#include <autoware_perception_msgs/Semantic.h>
#include <autoware_perception_msgs/Shape.h>
#include <autoware_planning_msgs/Route.h>
#include <autoware_system_msgs/AutowareState.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <scenario_api/scenario_api_coordinate_manager.h>
#include <scenario_api/scenario_calc_dist_utils.h>
#include <scenario_api_autoware/scenario_api_autoware.h>
#include <scenario_api_simulator/scenario_api_simulator.h>
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
#include <uuid_msgs/UniqueID.h>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <chrono>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace lanelet
{
class Lanelet;
class LaneletMap;
using LaneletMapPtr = std::shared_ptr<LaneletMap>;
namespace routing
{
class RoutingGraph;
}
namespace traffic_rules
{
class TrafficRules;
}
}  // namespace lanelet

class ScenarioAPI
{
public:
  /**
   * @brief constructor
   */
  ScenarioAPI();

  /**
   * @brief destructor
   */
  ~ScenarioAPI();

  //****************************************************************************************************public API
  //****************************************************************************************************public API
  //****************************************************************************************************public API
  //****************************************************************************************************public API
  //****************************************************************************************************public API
  //****************************************************************************************************public API

  // basic API
  bool setEgoCarName(const std::string & name);
  bool isEgoCarName(const std::string & name);
  bool isAPIReady();    // check ready for all callback function
  bool waitAPIReady();  // wait until ready
  bool updateState();   // update state //TODO

  // start API
  bool sendStartPoint(
    const double x, const double y, const double z, const double yaw, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendStartPoint(
    const geometry_msgs::Pose pose, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendStartPoint(
    const double p_x, const double p_y, const double p_z, const double o_x, const double o_y,
    const double o_z, const double o_w, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendGoalPoint(
    const double x, const double y, const double z, const double yaw, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendGoalPoint(
    const geometry_msgs::Pose pose, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendGoalPoint(
    const std::string & name, const geometry_msgs::Pose pose, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendGoalPoint(
    const double p_x, const double p_y, const double p_z, const double o_x, const double o_y,
    const double o_z, const double o_w, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendCheckPoint(
    const double x, const double y, const double z, const double yaw, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendCheckPoint(
    const geometry_msgs::Pose pose, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendCheckPoint(
    const std::string & name, const geometry_msgs::Pose pose, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendCheckPoint(
    const double p_x, const double p_y, const double p_z, const double o_x, const double o_y,
    const double o_z, const double o_w, const bool wait_ready = true,
    const std::string & frame_type = "Center");
  bool sendStartVelocity(const double velocity);
  bool sendEngage(const bool engage);
  bool waitAutowareInitialize();
  bool setMaxSpeed(double velocity);

  // coordinate API
  bool setFrameId(
    std::string frame_id, const double x, const double y, const double z, const double yaw);
  bool setFrameId(std::string frame_id, const geometry_msgs::Pose pose);
  bool setFrameId(
    std::string frame_id, const double p_x, const double p_y, const double p_z, const double o_x,
    const double o_y, const double o_z, const double o_w);
  geometry_msgs::Pose getRelativePose(
    std::string frame_id, const double x, const double y, const double z, const double yaw);
  geometry_msgs::Pose getRelativePose(std::string frame_id, const geometry_msgs::Pose pose);
  geometry_msgs::Pose getRelativePose(
    std::string frame_id, const double p_x, const double p_y, const double p_z, const double o_x,
    const double o_y, const double o_z, const double o_w);

  // basic self vehicle API
  Pose2D getCurrentPose();
  geometry_msgs::PoseStamped getCurrentPoseRos();
  double getVelocity();
  double getAccel();
  double getJerk();
  double getMoveDistance();
  bool isStopped(double thresh_velocity = 10e-3);
  bool isInArea(
    double x, double y, double yaw, double dist_thresh,
    double delta_yaw_thresh);  //!< @brief in designated area or not
  bool isInArea(
    geometry_msgs::Pose pose, double dist_thresh,
    double delta_yaw_thresh);  //!< @brief in designated area or not
  bool isInArea(
    double p_x, double p_y, double o_x, double o_y, double o_z, double o_w, double dist_thresh,
    double delta_yaw_thresh);  //!< @brief in designated area or not

  // additonal self vehicle API
  bool willLaneChange();  // TODO //!< @brief t4b try to change lanes or not
  bool getLeftBlinker();
  bool getRightBlinker();
  bool approveLaneChange(bool approve_lane_change);

  // lane API
  bool getCurrentLaneID(
    int & current_id, double max_dist = 3.0, double max_deleta_yaw = M_PI / 4.0);
  bool isChangeLaneID();  // future work: // TODO

  bool getDistancefromCenterLine(double & dist_from_center_line);
  bool isInLane();

  // obstacle API
  double getMinimumDistanceToObstacle(bool consider_height);

  // NPC API
  bool addNPC(
    const std::string & npc_type, const std::string & name, const double x, const double y,
    const double z, const double yaw, const double velocity, const bool stop_by_vehicle = false,
    const std::string & frame_type = "Center");
  bool addNPC(
    const std::string & npc_type, const std::string & name, geometry_msgs::Pose pose,
    const double velocity, const bool stop_by_vehicle = false,
    const std::string & frame_type = "Center");
  bool addNPC(
    const std::string & npc_type, const std::string & name, const double p_x, const double p_y,
    const double p_z, const double o_x, const double o_y, const double o_z, const double o_w,
    const double velocity, const bool stop_by_vehicle = false,
    const std::string & frame_type = "Center");
  bool changeNPCVelocity(const std::string & name, const double velocity);
  bool changeNPCAccelMin(const std::string & name, const double accel);
  bool changeNPCAccelMax(const std::string & name, const double accel);
  bool changeNPCVelocityWithAccel(
    const std::string & name, const double velocity, const double accel);
  bool changeNPCConsiderVehicle(const std::string & name, const bool consider_ego_vehicle);
  bool changeNPCLaneChangeLeft(const std::string & name);
  bool changeNPCLaneChangeRight(const std::string & name);
  bool changeNPCLaneChange(const std::string & name, const int target_lane_id);
  bool changeNPCUturn(const std::string & name);
  bool changeNPCTurnLeft(const std::string & name);
  bool changeNPCTurnRight(const std::string & name);
  bool changeNPCNoTurn(const std::string & name);
  bool changeNPCIgnoreLane(const std::string & name);
  bool deleteNPC(const std::string & name);
  bool calcDistToNPC(double & dist_to_npc, const std::string & name);
  bool calcDistToNPCFromNPC(
    double & distance, const std::string & npc1_name, const std::string & npc2_name);
  bool finishNPCLaneChange(const std::string & name, bool * finish_lane_change);
  bool finishNPCVelocityChange(const std::string & name, bool * finish_velocity_change);
  bool getNPCVelocity(const std::string id, double * velocity);
  bool getNPCAccel(const std::string id, double * accel);

  std::vector<std::string> getNpcList();
  bool isNpcExist(const std::string & name);

  // traffic light API
  bool setTrafficLightColor(
    const int traffic_id, const std::string traffic_color,
    const bool use_traffic_light);  // future work(simulator) //TODO
  bool setTrafficLightArrow(
    const int traffic_id, const std::string traffic_arrow,
    const bool use_traffic_light);  // future work(simulator) //TODO
  bool resetTrafficLightColor(
    const int traffic_id,
    const bool use_traffic_light);  // future work(simulator) //TODO
  bool resetTrafficLightArrow(
    const int traffic_id,
    const bool use_traffic_light);  // future work(simulator) //TODO

  bool getTrafficLightColor(
    const int traffic_id, std::string * traffic_color,
    const bool use_traffic_light);  // future work(simulator) //TODO
  bool getTrafficLightArrow(
    const int traffic_id, std::vector<std::string> * const traffic_arrow,
    const bool use_traffic_light);  // future work(simulator) //TODO

  bool getTrafficLineCenterPose(const int traffic_relation_id, geometry_msgs::Pose & line_pose);
  bool getDistanceToTrafficLight(const int traffic_relation_id, double & distance);
  bool getDistanceToTrafficLine(const int traffic_relation_id, double & distance);
  bool checkOverTrafficLine(const int traffic_relation_id, bool & over_line);

  //util API
  double getDistanceToArea(
    const std::string &, const geometry_msgs::Pose, const std::string & = "Center");

  bool isObjectInArea(
    const std::string & name, const geometry_msgs::Pose pose, const double dist_thresh,
    const double delta_yaw_thresh,
    const std::string & frame_type =
      "Center");  //!< @brief object(ego-car, NPC) is in designated area or not
  geometry_msgs::Pose genPoseROS(const double x, const double y, const double z, const double yaw);
  geometry_msgs::Pose genPoseROS(
    const double p_x, const double p_y, const double p_z, const double o_x, const double o_y,
    const double o_z, const double o_w);

  //****************************************************************************************************public API
  //****************************************************************************************************public API
  //****************************************************************************************************public API
  //****************************************************************************************************public API
  //****************************************************************************************************public API
  //****************************************************************************************************public API

private:
  ros::NodeHandle nh_;         //!< @brief ros node handle
  ros::NodeHandle pnh_;        //!< @brief private ros node handle
  ros::ServiceClient client_;  //!< @brief private ros service client
  ros::Subscriber sub_state_;  //!< @brief topic subscriber for autoware state
  ros::Subscriber sub_pcl_;    //!< @brief topic subscriber for pcl
  ros::Subscriber sub_map_;    //!< @brief topic subscriber for map
  ros::Subscriber
    sub_route_;  //!< @brief topic subscriber for current route (for check of autoware ready)
  ros::Subscriber sub_twist_;          //!< @brief topic subscriber for twist
  ros::Subscriber sub_turn_signal_;    //!< @brief topic subscriber for turn signal(blinker)
  ros::Timer timer_control_;           //!< @brief timer for getting self-position
  ros::Publisher pub_start_point_;     //!< @brief topic pubscriber for start point
  ros::Publisher pub_goal_point_;      //!< @brief topic pubscriber for goal point
  ros::Publisher pub_check_point_;     //!< @brief topic pubscriber for check point
  ros::Publisher pub_start_velocity_;  //!< @brief topic @publisher for initial velocity
  ros::Publisher pub_max_velocity_;    //!< @brief topic pubscriber for max velocity
  ros::Publisher pub_engage_;          //!< @brief topic pubscriber for engage
  ros::Publisher pub_object_info_;     //!<@brief topic pubscriber for npc

  std::shared_ptr<ScenarioAPISimulator> simulator_api_;
  std::shared_ptr<ScenarioAPIAutoware> autoware_api_;
  std::shared_ptr<ScenarioAPICoordinateManager> coordinate_api_;

  std::string ego_car_name_ = "";
  bool is_autoware_ready_initialize;
  bool is_autoware_ready_routing;
  std::string autoware_state_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  // function for start API
  bool waitState(const std::string state);

  // function for basic self vehicle API
  double getAccel(
    const std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr,
    const std::shared_ptr<geometry_msgs::TwistStamped> previous_twist_ptr);

  double getJerk(
    const std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr,
    const std::shared_ptr<geometry_msgs::TwistStamped> previous_twist_ptr,
    const std::shared_ptr<geometry_msgs::TwistStamped> second_previous_twist_ptr);

  // function for additonal self vehicle API
  bool getLeftBlinker(std::shared_ptr<autoware_vehicle_msgs::TurnSignal> turn_signal_ptr);
  bool getRightBlinker(std::shared_ptr<autoware_vehicle_msgs::TurnSignal> turn_signal_ptr);

  // function for lane API
  bool getCurrentLaneID(
    int & current_id, const std::shared_ptr<geometry_msgs::PoseStamped> & current_pose,
    const lanelet::LaneletMapPtr & lanelet_map_ptr, double max_dist, double max_deleta_yaw);
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

  // function for obstacle API
  double calcMinimumDistanceToObstacle(
    const std::shared_ptr<sensor_msgs::PointCloud2> & pointcloud, const bool consider_height);

  // NPC API
  bool getNPC(
    const std::string & name, geometry_msgs::Pose & object_pose,
    geometry_msgs::Twist & object_twist, geometry_msgs::Vector3 & object_size,
    std::string & object_name);
};

#endif  // SCENARIO_API_SCENARIO_API_CORE_H_INCLUDED
