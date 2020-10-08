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
#include <scenario_api_utils/scenario_api_utils.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "scenario_api");
  ScenarioAPI obj;

  /* sample code */
  obj.setEgoCarName("EgoCar");
  obj.waitAutowareInitialize();  // wait for initialization of autoware
  geometry_msgs::Pose ego_car_start_pose = obj.genPoseROS(3820.9, 73772.9, 0.0, 3.58);
  obj.sendStartPoint(ego_car_start_pose);  // send start point(x, y, z, yaw)
  obj.setFrameId("/initialpose", ego_car_start_pose);
  geometry_msgs::Pose ego_car_goal_pose = obj.genPoseROS(3714.2, 73741.65, 0.0, 2.01);
  obj.sendGoalPoint("EgoCar", ego_car_goal_pose, true, "Front");  // send goal point(name, pose)
  geometry_msgs::Pose ego_car_checkpoint_pose = obj.genPoseROS(3717.9, 73684.4, 0.0, 2.01);
  obj.sendCheckPoint(
    "EgoCar",
    ego_car_checkpoint_pose);  // send check point(pose) (optional; it should be called after sendGoalPoint)
  obj.setMaxSpeed(20.0);        // send max velocity of autoware (m/s)
  obj.sendStartVelocity(7.0);   // send initial velocity
  obj.approveLaneChange(true);  // send approval of lane change (default in autoware: true)

  obj.waitAPIReady();  // wait for API ready

  /*generate and modify NPC */
  // addNPC(const std::string& npc_type, const uint16_t id,
  // const geometry_msgs::Pose pose, const double velocity);
  geometry_msgs::Pose ped0_pose = obj.genPoseROS(3715.7, 73773.87, 0.0, 0.474);
  obj.addNPC("pedestrian", "ped0", ped0_pose, 0.2);
  obj.changeNPCVelocityWithAccel("ped0", 3, 1.0);

  geometry_msgs::Pose bicycle0_pose = obj.genPoseROS(3819.2, 73799.2, 0.0, -1.096);
  obj.addNPC("bicycle", "bicycle0", bicycle0_pose, 5.0);

  geometry_msgs::Pose car0_pose = obj.genPoseROS(3819.2, 73799.2, 0.0, -1.096);
  obj.addNPC("car", "car0", car0_pose, 0.0);

  // generate NPC to change lane(left, right)
  geometry_msgs::Pose car_lc_0_pose = obj.genPoseROS(3701.0, 73764.7, 0.0, 0.467);
  obj.addNPC("car", "car_lc_0", car_lc_0_pose, 1.0);
  obj.changeNPCLaneChangeRight("car_lc_0");
  geometry_msgs::Pose car_lc_1_pose = obj.genPoseROS(3702.6, 73762.1, 0.0, 0.467);
  obj.addNPC("car", "car_lc_1", car_lc_1_pose, 2.0);
  obj.changeNPCLaneChangeLeft("car_lc_1");

  // generate NPC to change lane to target lane id
  geometry_msgs::Pose car_lc_2_pose = obj.genPoseROS(3724.5, 73773.2, 0.0, 0.467);
  obj.addNPC("car", "car_lc_2", car_lc_2_pose, 1.0);
  obj.changeNPCLaneChange("car_lc_2", 34513);

  // generate NPC to accel/to turn right/stop by vehicle
  geometry_msgs::Pose car_acc_tr_0_pose = obj.genPoseROS(3702.6, 73762.1, 0.0, 0.467);
  obj.addNPC("car", "car_acc_tr_0", car_acc_tr_0_pose, 0.0);
  ros::Rate(10.0).sleep();                             //0.1s sleep
  obj.changeNPCConsiderVehicle("car_acc_tr_0", true);  //change parameter: stop by vehicle
  obj.changeNPCVelocityWithAccel(
    "car_acc_tr_0", 15.0, 2.0);  // std::string name, double velocity, double accel
  obj.changeNPCTurnRight("car_acc_tr_0");

  // generate NPC to do U-turn
  geometry_msgs::Pose car_uturn_0_pose = obj.genPoseROS(3746.8, 73685.0, 0.0, -2.67);
  obj.addNPC("car", "car_uturn_0", car_uturn_0_pose, 1.0);
  ros::Rate(0.5).sleep();  // 2s sleep
  obj.changeNPCUturn("car_uturn_0");
  obj.changeNPCVelocityWithAccel("car_uturn_0", 5.0, 1.0);

  // NPC generation in relative position
  geometry_msgs::Pose rel_pose = obj.genPoseROS(200.0, 0.0, 0.0, 0.0);
  auto pose = obj.getRelativePose("/initialpose", rel_pose);
  obj.addNPC("car", "car_rel_0", pose, 2.0);

  // generate NPC to follow route
  geometry_msgs::Pose car_fr_0_start_pose =
    obj.genPoseROS(3751.0, 73786.8, 0.0, 0.0, 0.0, 0.2361, 0.9717);
  geometry_msgs::Pose car_fr_0_goal_pose =
    obj.genPoseROS(3776.8, 73718.3, 0.0, 0.0, 0.0, 0.8536, 0.5209);
  obj.addNPC("car", "car_fr_0", car_fr_0_start_pose, 5.0);
  obj.sendGoalPoint("car_fr_0", car_fr_0_goal_pose, true);
  obj.changeNPCLaneChange("car_fr_0", 34507);

  // generate NPC to follow route with checkpoint
  geometry_msgs::Pose car_fr_1_start_pose =
    obj.genPoseROS(3751.0, 73786.8, 0.0, 0.0, 0.0, 0.2361, 0.9717);
  geometry_msgs::Pose car_fr_1_goal_pose =
    obj.genPoseROS(3721.4, 73721.8, 0.0, 0.0, 0.0, -0.9678, 0.2516);
  geometry_msgs::Pose car_fr_1_checkpoint_pose =
    obj.genPoseROS(3776.8, 73718.3, 0.0, 0.0, 0.0, 0.8536, 0.5209);
  obj.addNPC("car", "car_fr_1", car_fr_1_start_pose, 5.5);
  obj.sendGoalPoint("car_fr_1", car_fr_1_goal_pose, true);
  obj.sendCheckPoint("car_fr_1", car_fr_1_checkpoint_pose, true);
  obj.changeNPCLaneChange("car_fr_1", 34507);

  // send engage and start point
  obj.sendEngage(true);  // send engage(allow autoware to move)

  /* set traffic light color */  //#############################################################
  // setTrafficLight(int traffic_id, std::string traffic_color, bool use_traffic_light)
  // use_traffic_light is false: output detection result of traffic light
  // use_traffic_light is true: change traffic light color in simulator (**not implemented**)
  obj.setTrafficLightColor(34806, "green", false);   // add
  obj.setTrafficLightColor(34806, "red", false);     // modify
  obj.setTrafficLightColor(40000, "yellow", false);  // invalid
  obj.updateState();
  ros::Rate(0.3).sleep();  //3.0s sleep
  std::string color;
  std::vector<std::string> arrows;
  if (obj.getTrafficLightColor(34806, &color, false)) {
    ROS_INFO_STREAM("traffic:34806 color(RED) is " << color);
  } else {
    ROS_INFO_STREAM("failed to get traffic:34806 color");
  }
  obj.updateState();
  ros::Rate(0.3).sleep();  //3.0s sleep
  if (obj.getTrafficLightArrow(34806, &arrows, false)) {
    ROS_INFO_STREAM("traffic:34806 arrow size(NULL) is " << arrows.size());
  }
  obj.setTrafficLightArrow(34806, "left", false);
  obj.setTrafficLightArrow(34806, "up", false);
  if (obj.getTrafficLightArrow(34806, &arrows, false)) {
    ROS_INFO_STREAM("traffic:34806 arrow size(2) is " << arrows.size());
    for (const auto arrow : arrows) {
      ROS_INFO_STREAM("traffic:34806 arrow (left, up)) is " << arrow);
    }
  }
  obj.updateState();
  ros::Rate(0.3).sleep();  //3.0s sleep

  obj.resetTrafficLightArrow(34806, false);
  if (obj.getTrafficLightArrow(34806, &arrows, false)) {
    ROS_INFO_STREAM("traffic:34806 arrow size(0) is " << arrows.size());
    for (const auto arrow : arrows) {
      ROS_INFO_STREAM("traffic:34806 arrow (null)) is " << arrow);
    }
  }
  obj.updateState();
  ros::Rate(0.3).sleep();  //3.0s sleep
  obj.setTrafficLightArrow(34806, "right", false);
  obj.setTrafficLightArrow(34806, "down", false);
  obj.resetTrafficLightColor(34806, false);
  obj.setTrafficLightColor(34806, "yellow", false);  // modify
  obj.updateState();
  //$rostopic echo /perception/traic_light_recognition/traffic_light_states
  //result: 34802, 34836->(3,5,7)
  ros::Rate(0.3).sleep();        //3.0s sleep
  /* set traffic light color */  //#############################################################

  geometry_msgs::Pose line_pose;
  if (obj.getTrafficLineCenterPose(34806, line_pose)) {
    ROS_INFO_STREAM("traffic:34806 pose is " << line_pose);
  } else {
    ROS_INFO_STREAM("failed to get traffic:34806 pose");
  }
  ros::Rate(0.5).sleep();  //2.0s sleep

  while (ros::ok()) {
    obj.updateState();  // Unimplemented. update simulator step(?)
    ROS_INFO_STREAM("*************************************");
    ROS_INFO_STREAM("velocity:" << obj.getVelocity());
    ROS_INFO_STREAM("accel:" << obj.getAccel());
    ROS_INFO_STREAM("jerk:" << obj.getJerk());
    ROS_INFO_STREAM("total move distance:" << obj.getMoveDistance());
    ROS_INFO_STREAM(
      "getMinimumDistanceToObstacle:"
      << obj.getMinimumDistanceToObstacle(false));  // false: do not consider z-axis of pointcloud
    int current_id;
    double dist1, dist2, dist3;
    bool over_line;
    if (obj.getCurrentLaneID(current_id)) ROS_INFO_STREAM("getCurrentLaneID:" << current_id);
    if (obj.getDistanceToTrafficLight(34806, dist1)) {
      ROS_INFO_STREAM("dist to traffic light: 34806 is " << dist1);
    }
    if (obj.getDistanceToTrafficLine(34806, dist2)) {
      ROS_INFO_STREAM("dist to traffic light line: 34806 is " << dist2);
    } else if (obj.getDistancefromCenterLine(dist3))
      ROS_INFO_STREAM("getDistancefromCenterLine:" << dist3);
    if (obj.checkOverTrafficLine(34806, over_line)) {
      ROS_INFO_STREAM("over stop line (34806)? :" << over_line);
    }
    ROS_INFO_STREAM("isInLane:" << obj.isInLane());  // base_link is in lane or not
    ROS_INFO_STREAM("LeftBlinker:" << obj.getLeftBlinker());
    ROS_INFO_STREAM("RightBlinker:" << obj.getRightBlinker());
    ROS_INFO_STREAM("WillLaneChange" << obj.willLaneChange());  // temporary implementation
    Pose2D p = obj.getCurrentPose();
    ROS_INFO_STREAM("selfpose:" << p.x << "," << p.y << "," << p.yaw);
    geometry_msgs::PoseStamped p2 = obj.getCurrentPoseRos();
    ROS_INFO_STREAM("selfpose:" << p2.pose.position << "," << p2.pose.orientation);
    double dist_npc;
    if (obj.calcDistToNPC(dist_npc, "car0")) {
      ROS_INFO_STREAM("distance to npc:car0:" << dist_npc);
    }
    double npc_vel;
    double npc_acc;
    if (obj.getNPCVelocity("car_acc_tr_0", &npc_vel)) {
      ROS_INFO_STREAM("velocity of npc:car_acc_tr_0:" << npc_vel);
    }
    if (obj.getNPCAccel("car_acc_tr_0", &npc_acc)) {
      ROS_INFO_STREAM("accel of npc:car_acc_tr_0:" << npc_acc);
    }
    ROS_INFO_STREAM(
      "NPC:car_acc_tr_0 in area?" << obj.isObjectInArea(
        "car_acc_tr_0", car_acc_tr_0_pose, 10.0, M_PI / 4.0));
    bool finish_lane_change;
    if (obj.finishNPCLaneChange("car_uturn_0", &finish_lane_change)) {
      ROS_INFO_STREAM("car_uturn_0: finish lane change?" << finish_lane_change);
    }
    bool finish_vel_change;
    if (obj.finishNPCVelocityChange("car_uturn_0", &finish_vel_change)) {
      ROS_INFO_STREAM("car_uturn_0: finish velocity change?" << finish_vel_change);
    }

    //   bool isInArea(double x, double y, double yaw, double dist_thresh, double delta_yaw_thresh)
    if (
      obj.isStopped() and
      obj.isObjectInArea("EgoCar", ego_car_goal_pose, 1.0, M_PI / 10.0, "Front")) {
      // bool deleteNPC(const uint16_t id)
      obj.deleteNPC("ped0");
      // bool modifyNPC(const uint16_t id, double velocity)
      obj.changeNPCAccelMax("car_rel_0", 1.0);   //set accel max
      obj.changeNPCVelocity("car_rel_0", 10.0);  // modify NPC velocity
      break;
    }
  }

  ROS_INFO_STREAM("scenario end");
  return 0;
};
