#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import time
from rosgraph_msgs.msg import Clock

class RosTimePublisher(Node):
    def __init__(self):
        super().__init__('ros_time_publisher')
        use_sim_time_param = rclpy.parameter.Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            False)
        self.set_parameters([use_sim_time_param])
        self.declare_parameter("time_rate", 1.0)
        self.time_rate_ = self.get_parameter("time_rate").get_parameter_value().double_value
        # rclpy does not have WallTime)
        self.system_clock_ = rclpy.clock.Clock()
        self.initial_time_ = self.system_clock_.now()
        self.current_ros_time_ = self.initial_time_
        self.pub_clock_ = self.create_publisher(
            Clock, "/clock", 1)
        self.timer_ = self.create_timer(1e-03, self.timerCallback, clock=self.system_clock_)

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
