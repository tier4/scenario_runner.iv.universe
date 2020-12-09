#!/usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
import time
from rosgraph_msgs.msg import Clock

class RosTimePublisher(Node):
    def __init__(self):
        super().__init__('ros_time_publisher')
        time_param_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                                    description='clock publishing rate compared to real time')
        self.declare_parameter("time_rate", 1.0, time_param_descriptor)
        self.time_rate_ = self.get_parameter("time_rate").get_parameter_value().double_value
        self.system_clock_ = rclpy.clock.Clock()
        self.initial_time_ = self.system_clock_.now()
        self.current_ros_time_ = self.initial_time_
        self.pub_clock_ = self.create_publisher(
            Clock, "/clock", 1)
        self.timer_ = self.create_timer(1e-03, self.timerCallback)

    def timerCallback(self):
        # calculate current ros time with time_rate
        current_walltime = self.system_clock_.now()
        dt_walltime = current_walltime - self.initial_time_
        dt_ros_time_ns = dt_walltime.nanoseconds * self.time_rate_
        self.current_ros_time_ = self.initial_time_ + rclpy.duration.Duration(nanoseconds=dt_ros_time_ns)

        # publish clock
        clock_msg = Clock()
        clock_msg.clock = self.current_ros_time_.to_msg()
        try:
            self.pub_clock_.publish(clock_msg)
        except:
            self.get_logger().error("cannot publish clock")

def main(args=None):
    rclpy.init(args=args)
    node = RosTimePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
