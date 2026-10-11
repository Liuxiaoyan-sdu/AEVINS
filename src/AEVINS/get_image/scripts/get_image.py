#!/usr/bin/env python
#rosrun your_package_name get_image.py _topic_name:=/your/image/topic _save_dir:=/path/to/your/save_directory
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class ImageSaver:
    def __init__(self, topic_name, save_dir):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.callback)
        self.image_count = 0
        self.save_dir = save_dir

        # 创建保存图像的目录（如果不存在）
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # 构建保存图像的文件名
        image_name = f"image_{self.image_count:04d}.jpg"
        image_path = os.path.join(self.save_dir, image_name)
        
        # 保存图像
        cv2.imwrite(image_path, cv_image)
        
        # 输出保存的图像文件名
        rospy.loginfo(f"Saved image {image_path}")
        
        # 更新图像计数器
        self.image_count += 1

def main():
    # 初始化ROS节点
    rospy.init_node('image_saver', anonymous=True)

    # 获取参数：图像话题名称和保存目录
    topic_name = rospy.get_param('~topic_name', '/camera/image')
    save_dir = rospy.get_param('~save_dir', '/home/lxy/tools_ws/image/event_cl')

    # 创建ImageSaver实例
    image_saver = ImageSaver(topic_name, save_dir)
    
    try:
        # 保持节点运行，直到被中断
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
