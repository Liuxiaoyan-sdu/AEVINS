#!/usr/bin/env python
"""
DVL Frame Transformer Node
功能：
- 订阅DVL发布的Odometry消息（/dvl/odometry）
- 将其从dvl_link坐标系转换到world坐标系
- 发布转换后的Odometry消息到新话题（/dvl/odometry_world）

依赖：
- ROS (rospy)
- tf2_ros
"""

import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class DVLFrameTransformer:
    def __init__(self):
        rospy.init_node("dvl_frame_transformer")

        # Publisher
        self.odometry_pub = rospy.Publisher("/dvl/odometry_world", Odometry, queue_size=10)

        # Subscriber
        rospy.Subscriber("/dvl/odometry", Odometry, self.odometry_callback)

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("DVL Frame Transformer node started")

    def odometry_callback(self, msg):
        try:
            # 获取从 dvl_link 到 world 的变换
            transform = self.tf_buffer.lookup_transform("world", "dvl_link", rospy.Time(0))

            # 创建新的Odometry消息
            transformed_odom = Odometry()
            transformed_odom.header.stamp = msg.header.stamp
            transformed_odom.header.frame_id = "world"
            transformed_odom.child_frame_id = "dvl_link"

            # 变换位置
            transformed_position = tf_conversions.transformations.translation_matrix(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            )
            transformed_position = tf_conversions.transformations.concatenate_matrices(
                transform.transform.translation, transformed_position
            )

            transformed_odom.pose.pose.position.x = transformed_position[0]
            transformed_odom.pose.pose.position.y = transformed_position[1]
            transformed_odom.pose.pose.position.z = transformed_position[2]

            # 变换姿态
            transformed_orientation = tf_conversions.transformations.quaternion_multiply(
                [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w],
                [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            )

            transformed_odom.pose.pose.orientation.x = transformed_orientation[0]
            transformed_odom.pose.pose.orientation.y = transformed_orientation[1]
            transformed_odom.pose.pose.orientation.z = transformed_orientation[2]
            transformed_odom.pose.pose.orientation.w = transformed_orientation[3]

            # 发布转换后的Odometry
            self.odometry_pub.publish(transformed_odom)

        except tf2_ros.LookupException as e:
            rospy.logwarn(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn(f"Transform extrapolation failed: {e}")

if __name__ == "__main__":
    try:
        transformer = DVLFrameTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("DVL Frame Transformer node shutdown")