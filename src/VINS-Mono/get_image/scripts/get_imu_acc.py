import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import quaternion_matrix

class OdometryToTransform:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('odometry_to_transform', anonymous=True)
        
        # 创建Odometry消息订阅者
        self.odom_sub = rospy.Subscriber('/dvl/position', Odometry, self.odom_callback)
        
        # 设置发布频率
        self.rate = rospy.Rate(10)  # 10Hz

    def odom_callback(self, msg):
        # 获取位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # 获取四元数姿态
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # 直接从四元数生成4x4变换矩阵，并提取3x3旋转矩阵
        rot_matrix = quaternion_matrix(orientation_list)[:3, :3]
        
        # 创建4x4齐次变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rot_matrix
        transform_matrix[:3, 3] = [x, y, z]
        
        # 输出变换矩阵
        rospy.loginfo("变换矩阵:\n%s", transform_matrix)

    def run(self):
        # 保持ROS节点运行
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = OdometryToTransform()
        node.run()
    except rospy.ROSInterruptException:
        pass