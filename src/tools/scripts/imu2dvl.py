#!/usr/bin/env python
'''
DVL数据转换节点
功能：将DVL发布的Imu消息转换为标准ROS消息格式
'''

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math

class DVLConverter:
    def __init__(self):
        rospy.init_node("dvl_converter")
        
        #  Publishers
        self.velocity_pub = rospy.Publisher("/dvl/velocity_converted", Vector3Stamped, queue_size=10)
        self.odometry_pub = rospy.Publisher("/dvl/odometry", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("/dvl/pose", PoseStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("/dvl/velocity", Imu, self.velocity_callback)
        rospy.Subscriber("/dvl/position", Imu, self.position_callback)
        
        # 初始化位置和姿态
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        rospy.loginfo("DVL Converter node started")
        
    def velocity_callback(self, msg):
        """
        处理DVL速度数据，转换为Vector3Stamped格式
        """
        try:
            # 创建Vector3Stamped消息
            velocity_msg = Vector3Stamped()
            velocity_msg.header = Header()
            velocity_msg.header.stamp = msg.header.stamp  # 使用接收到的数据的时间戳
            velocity_msg.header.frame_id = "dvl_link"
            
            # 从linear_acceleration字段提取速度数据
            velocity_msg.vector.x = msg.linear_acceleration.x  # vx
            velocity_msg.vector.y = msg.linear_acceleration.y  # vy
            velocity_msg.vector.z = msg.linear_acceleration.z  # vz
            
            # 发布转换后的速度消息
            self.velocity_pub.publish(velocity_msg)
            
            rospy.logdebug(f"Velocity published: vx={velocity_msg.vector.x:.3f}, "
                          f"vy={velocity_msg.vector.y:.3f}, vz={velocity_msg.vector.z:.3f}")
                          
        except Exception as e:
            rospy.logerr(f"Error in velocity conversion: {str(e)}")
    
    def position_callback(self, msg):
        """
        处理DVL位置和姿态数据，转换为Odometry和PoseStamped格式
        """
        try:
            # 提取位置数据
            self.position_x = msg.orientation.x  # x position
            self.position_y = msg.orientation.y  # y position
            self.position_z = msg.orientation.z  # z position
            
            # 提取欧拉角
            self.roll = msg.angular_velocity.x   # roll
            self.pitch = msg.angular_velocity.y  # pitch
            self.yaw = msg.angular_velocity.z    # yaw
            
            # 发布里程计数据
            self.publish_odometry(msg.header.stamp)
            
            # 发布位姿数据
            self.publish_pose(msg.header.stamp)
            
            rospy.logdebug(f"Position: x={self.position_x:.3f}, y={self.position_y:.3f}, z={self.position_z:.3f}")
            rospy.logdebug(f"Orientation: roll={math.degrees(self.roll):.1f}°, "
                          f"pitch={math.degrees(self.pitch):.1f}°, yaw={math.degrees(self.yaw):.1f}°")
                          
        except Exception as e:
            rospy.logerr(f"Error in position conversion: {str(e)}")
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        将欧拉角转换为四元数
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q
    
    def publish_odometry(self, timestamp):
        """
        发布里程计数据
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "dvl_link"
        
        # 设置位置
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = self.position_z
        
        # 设置姿态（欧拉角转四元数）
        odom_msg.pose.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        
        # 设置协方差（简化处理）
        for i in range(36):
            if i % 7 == 0:  # 对角线元素
                odom_msg.pose.covariance[i] = 0.01
            else:
                odom_msg.pose.covariance[i] = 0.0
        
        self.odometry_pub.publish(odom_msg)
    
    def publish_pose(self, timestamp):
        """
        发布位姿数据
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "odom"
        
        # 设置位置
        pose_msg.pose.position.x = self.position_x
        pose_msg.pose.position.y = self.position_y
        pose_msg.pose.position.z = self.position_z
        
        # 设置姿态（欧拉角转四元数）
        pose_msg.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        
        self.pose_pub.publish(pose_msg)

def main():
    try:
        converter = DVLConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("DVL Converter node shutdown")
    except Exception as e:
        rospy.logerr(f"DVL Converter error: {str(e)}")

if __name__ == "__main__":
    main()