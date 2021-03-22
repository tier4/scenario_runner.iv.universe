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

#include "scenario_api/scenario_api_core.hpp"
#include "scenario_api_utils/scenario_api_utils.hpp"

namespace
{
// TODO(fred-apex-ai) how to print msg to stdout? https://answers.ros.org/question/365776/how-to-print-a-message-to-stdout-in-ros2/
// Once available, use the `to_yaml` function instead instead of this handcrafted operator
std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Pose & pose)
{
  out << "position = {x=" << pose.position.x << ", y=" << pose.position.y << ", z=" <<
    pose.position.z << "}" << std::endl;
  out << "orientation = {x=" << pose.orientation.x << ", y=" << pose.orientation.y << ", z=" <<
    pose.orientation.z << ", w=" << pose.orientation.w << "}" << std::endl;
  return out;
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto obj = std::make_shared<ScenarioAPI>();
  // TODO(fred-apex-ai) should not necessary upon refactoring
  obj->init();

  /* sample code */
  obj->setEgoCarName("EgoCar");
  obj->waitAutowareInitialize();  // wait for initialization of autoware
  geometry_msgs::msg::Pose ego_car_start_pose = obj->genPoseROS(3820.9, 73772.9, 0.0, 3.58);
  obj->sendStartPoint(ego_car_start_pose);  // send start point(x, y, z, yaw)
  obj->setFrameId("/initialpose", ego_car_start_pose);
  geometry_msgs::msg::Pose ego_car_goal_pose = obj->genPoseROS(3714.2, 73741.65, 0.0, 2.01);
  obj->sendGoalPoint("EgoCar", ego_car_goal_pose, true, "Front");  // send goal point(name, pose)
  geometry_msgs::msg::Pose ego_car_checkpoint_pose = obj->genPoseROS(3717.9, 73684.4, 0.0, 2.01);
  obj->sendCheckPoint(
    "EgoCar",
    ego_car_checkpoint_pose);  // send check point(pose) (optional; it should be called after sendGoalPoint)
  obj->setMaxSpeed(20.0);        // send max velocity of autoware (m/s)
  obj->sendStartVelocity(7.0);   // send initial velocity
  obj->approveLaneChange(true);  // send approval of lane change (default in autoware: true)

  obj->waitAPIReady();  // wait for API ready

  /*generate and modify NPC */
  // addNPC(const std::string& npc_type, const uint16_t id,
  // const geometry_msgs::msg::Pose pose, const double velocity);
  geometry_msgs::msg::Pose ped0_pose = obj->genPoseROS(3715.7, 73773.87, 0.0, 0.474);
  obj->addNPC("pedestrian", "ped0", ped0_pose, 0.2);
  obj->changeNPCVelocityWithAccel("ped0", 3, 1.0);

  geometry_msgs::msg::Pose bicycle0_pose = obj->genPoseROS(3819.2, 73799.2, 0.0, -1.096);
  obj->addNPC("bicycle", "bicycle0", bicycle0_pose, 5.0);

  geometry_msgs::msg::Pose car0_pose = obj->genPoseROS(3819.2, 73799.2, 0.0, -1.096);
  obj->addNPC("car", "car0", car0_pose, 0.0);

  // generate NPC to change lane(left, right)
  geometry_msgs::msg::Pose car_lc_0_pose = obj->genPoseROS(3701.0, 73764.7, 0.0, 0.467);
  obj->addNPC("car", "car_lc_0", car_lc_0_pose, 1.0);
  obj->changeNPCLaneChangeRight("car_lc_0");
  geometry_msgs::msg::Pose car_lc_1_pose = obj->genPoseROS(3702.6, 73762.1, 0.0, 0.467);
  obj->addNPC("car", "car_lc_1", car_lc_1_pose, 2.0);
  obj->changeNPCLaneChangeLeft("car_lc_1");

  // generate NPC to change lane to target lane id
  geometry_msgs::msg::Pose car_lc_2_pose = obj->genPoseROS(3724.5, 73773.2, 0.0, 0.467);
  obj->addNPC("car", "car_lc_2", car_lc_2_pose, 1.0);
  obj->changeNPCLaneChange("car_lc_2", 34513);

  // generate NPC to accel/to turn right/stop by vehicle
  geometry_msgs::msg::Pose car_acc_tr_0_pose = obj->genPoseROS(3702.6, 73762.1, 0.0, 0.467);
  obj->addNPC("car", "car_acc_tr_0", car_acc_tr_0_pose, 0.0);
  rclcpp::Rate(10.0).sleep();                           //0.1s sleep
  obj->changeNPCConsiderVehicle("car_acc_tr_0", true);  //change parameter: stop by vehicle
  obj->changeNPCVelocityWithAccel(
    "car_acc_tr_0", 15.0, 2.0);  // std::string name, double velocity, double accel
  obj->changeNPCTurnRight("car_acc_tr_0");

  // generate NPC to do U-turn
  geometry_msgs::msg::Pose car_uturn_0_pose = obj->genPoseROS(3746.8, 73685.0, 0.0, -2.67);
  obj->addNPC("car", "car_uturn_0", car_uturn_0_pose, 1.0);
  rclcpp::Rate(0.5).sleep();  // 2s sleep
  obj->changeNPCUturn("car_uturn_0");
  obj->changeNPCVelocityWithAccel("car_uturn_0", 5.0, 1.0);

  // NPC generation in relative position
  geometry_msgs::msg::Pose rel_pose = obj->genPoseROS(200.0, 0.0, 0.0, 0.0);
  auto pose = obj->getRelativePose("/initialpose", rel_pose);
  obj->addNPC("car", "car_rel_0", pose, 2.0);

  // generate NPC to follow route
  geometry_msgs::msg::Pose car_fr_0_start_pose =
    obj->genPoseROS(3751.0, 73786.8, 0.0, 0.0, 0.0, 0.2361, 0.9717);
  geometry_msgs::msg::Pose car_fr_0_goal_pose =
    obj->genPoseROS(3776.8, 73718.3, 0.0, 0.0, 0.0, 0.8536, 0.5209);
  obj->addNPC("car", "car_fr_0", car_fr_0_start_pose, 5.0);
  obj->sendGoalPoint("car_fr_0", car_fr_0_goal_pose, true);
  obj->changeNPCLaneChange("car_fr_0", 34507);

  // generate NPC to follow route with checkpoint
  geometry_msgs::msg::Pose car_fr_1_start_pose =
    obj->genPoseROS(3751.0, 73786.8, 0.0, 0.0, 0.0, 0.2361, 0.9717);
  geometry_msgs::msg::Pose car_fr_1_goal_pose =
    obj->genPoseROS(3721.4, 73721.8, 0.0, 0.0, 0.0, -0.9678, 0.2516);
  geometry_msgs::msg::Pose car_fr_1_checkpoint_pose =
    obj->genPoseROS(3776.8, 73718.3, 0.0, 0.0, 0.0, 0.8536, 0.5209);
  obj->addNPC("car", "car_fr_1", car_fr_1_start_pose, 5.5);
  obj->sendGoalPoint("car_fr_1", car_fr_1_goal_pose, true);
  obj->sendCheckPoint("car_fr_1", car_fr_1_checkpoint_pose, true);
  obj->changeNPCLaneChange("car_fr_1", 34507);

  // send engage and start point
  obj->sendEngage(true);  // send engage(allow autoware to move)

  /* set traffic light color */  //#############################################################
  // setTrafficLight(int traffic_id, std::string traffic_color, bool use_traffic_light)
  // use_traffic_light is false: output detection result of traffic light
  // use_traffic_light is true: change traffic light color in simulator (**not implemented**)
  obj->setTrafficLightColor(34806, "green", false);   // add
  obj->setTrafficLightColor(34806, "red", false);     // modify
  obj->setTrafficLightColor(40000, "yellow", false);  // invalid
  obj->updateState();
  rclcpp::Rate(0.3).sleep();  //3.0s sleep
  std::string color;
  std::vector<std::string> arrows;
  if (obj->getTrafficLightColor(34806, &color, false)) {
    RCLCPP_INFO_STREAM(obj->get_logger(), "traffic:34806 color(RED) is " << color);
  } else {
    RCLCPP_INFO_STREAM(obj->get_logger(), "failed to get traffic:34806 color");
  }
  obj->updateState();
  rclcpp::Rate(0.3).sleep();  //3.0s sleep
  if (obj->getTrafficLightArrow(34806, &arrows, false)) {
    RCLCPP_INFO_STREAM(obj->get_logger(), "traffic:34806 arrow size(NULL) is " << arrows.size());
  }
  obj->setTrafficLightArrow(34806, "left", false);
  obj->setTrafficLightArrow(34806, "up", false);
  if (obj->getTrafficLightArrow(34806, &arrows, false)) {
    RCLCPP_INFO_STREAM(obj->get_logger(), "traffic:34806 arrow size(2) is " << arrows.size());
    for (const auto & arrow : arrows) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "traffic:34806 arrow (left, up)) is " << arrow);
    }
  }
  obj->updateState();
  rclcpp::Rate(0.3).sleep();  //3.0s sleep

  obj->resetTrafficLightArrow(34806, false);
  if (obj->getTrafficLightArrow(34806, &arrows, false)) {
    RCLCPP_INFO_STREAM(obj->get_logger(), "traffic:34806 arrow size(0) is " << arrows.size());
    for (const auto & arrow : arrows) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "traffic:34806 arrow (null)) is " << arrow);
    }
  }
  obj->updateState();
  rclcpp::Rate(0.3).sleep();  //3.0s sleep
  obj->setTrafficLightArrow(34806, "right", false);
  obj->setTrafficLightArrow(34806, "down", false);
  obj->resetTrafficLightColor(34806, false);
  obj->setTrafficLightColor(34806, "yellow", false);  // modify
  obj->updateState();
  //$rostopic echo /perception/traffic_light_recognition/traffic_light_states
  //result: 34802, 34836->(3,5,7)
  rclcpp::Rate(0.3).sleep();     //3.0s sleep
  /* set traffic light color */  //#############################################################

  geometry_msgs::msg::Pose line_pose;
  if (obj->getTrafficLineCenterPose(34806, line_pose)) {
    RCLCPP_INFO_STREAM(obj->get_logger(), "traffic:34806 pose is " << line_pose);
  } else {
    RCLCPP_INFO_STREAM(obj->get_logger(), "failed to get traffic:34806 pose");
  }
  rclcpp::Rate(0.5).sleep();  //2.0s sleep

  while (rclcpp::ok()) {
    obj->updateState();  // Unimplemented. update simulator step(?)
    RCLCPP_INFO_STREAM(obj->get_logger(), "*************************************");
    RCLCPP_INFO_STREAM(obj->get_logger(), "velocity:" << obj->getVelocity());
    RCLCPP_INFO_STREAM(obj->get_logger(), "accel:" << obj->getAccel());
    RCLCPP_INFO_STREAM(obj->get_logger(), "jerk:" << obj->getJerk());
    RCLCPP_INFO_STREAM(obj->get_logger(), "total move distance:" << obj->getMoveDistance());
    RCLCPP_INFO_STREAM(
      obj->get_logger(),
      "getMinimumDistanceToObstacle:" << obj->getMinimumDistanceToObstacle(
        false));  // false: do not consider z-axis of pointcloud
    int current_id;
    double dist1, dist2, dist3;
    bool over_line;
    if (obj->getCurrentLaneID(current_id)) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "getCurrentLaneID:" << current_id);
    }
    if (obj->getDistanceToTrafficLight(34806, dist1)) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "dist to traffic light: 34806 is " << dist1);
    }
    if (obj->getDistanceToTrafficLine(34806, dist2)) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "dist to traffic light line: 34806 is " << dist2);
    } else if (obj->getDistancefromCenterLine(dist3)) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "getDistancefromCenterLine:" << dist3);
    }
    if (obj->checkOverTrafficLine(34806, over_line)) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "over stop line (34806)? :" << over_line);
    }
    RCLCPP_INFO_STREAM(
      obj->get_logger(), "isInLane:" << obj->isInLane());  // base_link is in lane or not
    RCLCPP_INFO_STREAM(obj->get_logger(), "LeftBlinker:" << obj->getLeftBlinker());
    RCLCPP_INFO_STREAM(obj->get_logger(), "RightBlinker:" << obj->getRightBlinker());
    RCLCPP_INFO_STREAM(
      obj->get_logger(), "WillLaneChange" << obj->willLaneChange());  // temporary implementation
    Pose2D p = obj->getCurrentPose();
    RCLCPP_INFO_STREAM(obj->get_logger(), "selfpose:" << p.x << "," << p.y << "," << p.yaw);
    geometry_msgs::msg::PoseStamped p2 = obj->getCurrentPoseRos();
    RCLCPP_INFO_STREAM(obj->get_logger(), "selfpose:" << p2.pose);
    double dist_npc;
    if (obj->calcDistToNPC(dist_npc, "car0")) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "distance to npc:car0:" << dist_npc);
    }
    double npc_vel;
    double npc_acc;
    if (obj->getNPCVelocity("car_acc_tr_0", &npc_vel)) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "velocity of npc:car_acc_tr_0:" << npc_vel);
    }
    if (obj->getNPCAccel("car_acc_tr_0", &npc_acc)) {
      RCLCPP_INFO_STREAM(obj->get_logger(), "accel of npc:car_acc_tr_0:" << npc_acc);
    }
    RCLCPP_INFO_STREAM(
      obj->get_logger(), "NPC:car_acc_tr_0 in area?" << obj->isObjectInArea(
        "car_acc_tr_0", car_acc_tr_0_pose, 10.0, M_PI / 4.0));
    bool finish_lane_change;
    if (obj->finishNPCLaneChange("car_uturn_0", &finish_lane_change)) {
      RCLCPP_INFO_STREAM(
        obj->get_logger(), "car_uturn_0: finish lane change?" << finish_lane_change);
    }
    bool finish_vel_change;
    if (obj->finishNPCVelocityChange("car_uturn_0", &finish_vel_change)) {
      RCLCPP_INFO_STREAM(
        obj->get_logger(), "car_uturn_0: finish velocity change?" << finish_vel_change);
    }

    //   bool isInArea(double x, double y, double yaw, double dist_thresh, double delta_yaw_thresh)
    if (
      obj->isStopped() &&
      obj->isObjectInArea("EgoCar", ego_car_goal_pose, 1.0, M_PI / 10.0, "Front"))
    {
      // bool deleteNPC(const uint16_t id)
      obj->deleteNPC("ped0");
      // bool modifyNPC(const uint16_t id, double velocity)
      obj->changeNPCAccelMax("car_rel_0", 1.0);   //set accel max
      obj->changeNPCVelocity("car_rel_0", 10.0);  // modify NPC velocity
      break;
    }
  }

  RCLCPP_INFO_STREAM(obj->get_logger(), "scenario end");
  return 0;
}
