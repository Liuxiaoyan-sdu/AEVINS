#!/usr/bin/env python3
"""
Simple test publisher that simulates backend optimization time and publishes
it to /vins_backend/opt_time (std_msgs/Float64). Use this to test the
feature tracker republishing to /feature_tracker/backend_timing.

Usage:
  rosrun tools publish_backend_time.py

Parameters (rosparam):
  ~period: publish period in seconds (default 1.0)
  ~base_ms: base optimization time in ms (default 10.0)
  ~jitter_ms: random jitter in ms added to base (default 5.0)
"""
import rospy
from std_msgs.msg import Float64
import random

if __name__ == '__main__':
    rospy.init_node('publish_backend_time')
    period = rospy.get_param('~period', 1.0)
    base = rospy.get_param('~base_ms', 10.0)
    jitter = rospy.get_param('~jitter_ms', 5.0)
    pub = rospy.Publisher('/vins_backend/opt_time', Float64, queue_size=10)
    rate = rospy.Rate(1.0/period)
    rospy.loginfo('publish_backend_time: publishing to /vins_backend/opt_time every %.3fs', period)
    while not rospy.is_shutdown():
        val = base + random.uniform(-jitter, jitter)
        msg = Float64()
        msg.data = max(0.0, val)
        pub.publish(msg)
        rate.sleep()
