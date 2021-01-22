import argparse
import os
import json
from pathlib import Path
from typing import Optional, Union

import launch

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription


def main():
    vehicle_model = 'lexus'
    vehicle_package = f'{vehicle_model}_description'
    vehicle_package_path = FindPackageShare(vehicle_package).find(vehicle_package)
    vehicle_info_param_path = Path(vehicle_package_path) / 'config' / 'vehicle_info.yaml'
    scenario_id = "0"

    #TODO: Edit
    # scenario_path = "/home/kosuke/scenario_tests/odd3/scenario/UC-003-0003/PSim_1/UC-003-0003_PSim_1_case1.yaml"
    scenario_path = "/home/kosuke/scenario_tests/odd2/scenario/UC-001-0003/PSim_1/UC-001-0003_PSim_1_case1.yaml"  #hamakita
    # scenario_path = "/home/kosuke/scenario_tests/odd3/scenario/UC-008-0005/PSim_1/UC-008-0005_PSim_1_case1.yaml"  #nishishinjuku_aisan
    log_output_base_dir = "/home/kosuke/AutowareArchitectureProposal/src/simulator/scenario_runner/tmp_launch"
    scenario_runner_args = {
                    'scenario_id': scenario_id,
                    'scenario_path': str(scenario_path),
                    'log_output_path': f'{log_output_base_dir}/{scenario_id}.json',
                    'json_dump_path': f'{log_output_base_dir}/result-of-{scenario_id}.json',
                    'camera_frame_id': 'camera5/camera_optical_link',
                    'initial_engage_state': False
                }
    scenario_runner = Node(
        package='scenario_runner',
        executable='scenario_runner_node',
        parameters=[
            scenario_runner_args,
            vehicle_info_param_path,
        ],
        remappings=[
            ("input/pointcloud", "/sensing/lidar/no_ground/pointcloud"),
            ("input/vectormap", "/map/vector_map"),
            ("input/route", "/planning/mission_planning/route"),
            ("input/autoware_state", "/autoware/state"),
            ("input/vehicle_twist", "/vehicle/status/twist"),
            ("input/signal_command", "/vehicle/status/turn_signal"),
            ("input/engage","/simulation/npc_simulator/engage"),
            ("input/ego_vehicle_pose","/current_pose"),
            ("output/start_point", "/initialpose"),
            ("output/initial_velocity", "/initialtwist"),
            ("output/goal_point", "/planning/mission_planning/goal"),
            ("output/check_point", "/planning/mission_planning/checkpoint"),
            ("output/autoware_engage", "/autoware/engage"),
            ("output/simulator_engage", "/vehicle/engage"),
            ("output/npc_simulator_engage", "/simulation/npc_simulator/engage"),  # noqa: E501
            ("output/limit_velocity", "/planning/scenario_planning/max_velocity"),  # noqa: E501
            ("output/object_info", "/simulation/npc_simulator/object_info"),
            ("output/traffic_detection_result", "/perception/traffic_light_recognition/traffic_light_states"),  # noqa: E501
            ("output/lane_change_permission", "/planning/scenario_planning/lane_driving/lane_change_approval"),  # noqa: E501
            ("output/dynamic_object_info" ,"/simulation/dummy_perception_publisher/object_info"),
            ("output/debug_object_info","/simulation/npc_simulator/ground_truth_object_info")]
    #  ),
        # sigterm_timeout=???, TO)
    )

    # launch_path= "/home/kosuke/AutowareArchitectureProposal/src/autoware/launcher/autoware_launch/launch/planning_simulator.launch.xml"
    # planning_simulator = IncludeLaunchDescription(
    #     FrontendLaunchDescriptionSource(
    #         launch_file_path=launch_path, parser=Parser()),  # noqa: E501

    #     launch_arguments=[("map_path" ,"/home/kosuke/scenario_tests/map/kashiwanoha"),
    #                       ("sensor_model",   "aip_xx1"),
    #                       ("vehicle_model",   "lexus")])

    # # When the scenario_runner exits, all nodes should be shut down.
    # def on_exit(event: Event, context: LaunchContext):
    #     return Shutdown(reason="Simulation complete")

    # shutdown_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=scenario_runner,
    #         on_exit=on_exit
    #     )
    # )

    # rosbag_record = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '-a'],
    #     output='screen'
    # )

    launch_desc = LaunchDescription([
                                    # planning_simulator,
                                    scenario_runner
                                    ]);
    ls = launch.LaunchService()
    ls.include_launch_description(launch_desc)
    ls.run()
    ls.shutdown()

if __name__ == "__main__":
    main()
