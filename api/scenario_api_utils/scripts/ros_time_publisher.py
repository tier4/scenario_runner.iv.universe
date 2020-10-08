#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from rosgraph_msgs.msg import Clock


class RosTimePublisher():

    def __init__(self):
        self.time_rate_ = rospy.get_param("~time_rate", 1.0)
        # rospy does not have WallTime)
        self.initial_time_ = rospy.Time.from_sec(time.time())
        self.current_ros_time_ = self.initial_time_
        self.pub_clock_ = rospy.Publisher(
            "/clock", Clock, tcp_nodelay=True, queue_size=1)
        rospy.Timer(rospy.Duration(1e-03), self.timerCallback)

    def timerCallback(self, event):
        # calculate current ros time with time_rate
        current_walltime = rospy.Time.from_sec(time.time())
        dt_walltime = current_walltime - self.initial_time_
        dt_walltime_ros = dt_walltime * self.time_rate_
        self.current_ros_time_ = self.initial_time_ + dt_walltime_ros

        # publish clock
        clock_msg = Clock()
        clock_msg.clock.secs = self.current_ros_time_.secs
        clock_msg.clock.nsecs = self.current_ros_time_.nsecs
        try:
            self.pub_clock_.publish(clock_msg)
        except:
            rospy.logerr("cannot publish clock")


def main():
    rospy.set_param("/use_sim_time", False)
    rospy.init_node("ros_time_publisher")
    RosTimePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
