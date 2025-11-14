#!/usr/bin/env python3


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading


class ImagePreprocessor:
    """图像预处理ROS节点"""
    
    def __init__(self):
        """初始化节点和参数"""
        rospy.init_node('image_preprocessor', anonymous=False)
        
        # 读取ROS参数
        self.input_topic = rospy.get_param('~input_topic', '/cam0/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/cam0/image_preprocessed')
        
        # 处理参数
        self.enable_denoise = rospy.get_param('~enable_denoise', True)
        self.denoise_h = rospy.get_param('~denoise_h', 10)
        self.denoise_template_size = rospy.get_param('~denoise_template_size', 7)
        self.denoise_search_size = rospy.get_param('~denoise_search_size', 21)
        
        self.enable_clahe = rospy.get_param('~enable_clahe', True)
        self.clahe_clip_limit = rospy.get_param('~clahe_clip_limit', 3.0)
        self.clahe_tile_size = rospy.get_param('~clahe_tile_size', 8)
        
        self.enable_bilateral = rospy.get_param('~enable_bilateral', False)
        self.bilateral_d = rospy.get_param('~bilateral_d', 9)
        self.bilateral_sigma_color = rospy.get_param('~bilateral_sigma_color', 75)
        self.bilateral_sigma_space = rospy.get_param('~bilateral_sigma_space', 75)
        
        self.enable_sharpen = rospy.get_param('~enable_sharpen', False)
        self.sharpen_amount = rospy.get_param('~sharpen_amount', 0.5)
        
        self.queue_size = rospy.get_param('~queue_size', 1)
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 创建CLAHE对象（避免每次重复创建）
        if self.enable_clahe:
            self.clahe = cv2.createCLAHE(
                clipLimit=self.clahe_clip_limit,
                tileGridSize=(self.clahe_tile_size, self.clahe_tile_size)
            )
        
        # 统计信息
        self.frame_count = 0
        self.last_time = rospy.Time.now()
        self.lock = threading.Lock()
        
        # 发布器
        self.image_pub = rospy.Publisher(
            self.output_topic, 
            Image, 
            queue_size=10
        )
        
        # 订阅器（放在最后，避免回调时对象未初始化完成）
        self.image_sub = rospy.Subscriber(
            self.input_topic,
            Image,
            self.image_callback,
            queue_size=self.queue_size,
            buff_size=2**24  # 16MB buffer
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("图像预处理节点已启动")
        rospy.loginfo(f"  输入话题: {self.input_topic}")
        rospy.loginfo(f"  输出话题: {self.output_topic}")
        rospy.loginfo(f"  去噪: {self.enable_denoise} (h={self.denoise_h})")
        rospy.loginfo(f"  CLAHE: {self.enable_clahe} (clip={self.clahe_clip_limit}, tile={self.clahe_tile_size})")
        rospy.loginfo(f"  双边滤波: {self.enable_bilateral}")
        rospy.loginfo(f"  锐化: {self.enable_sharpen}")
        rospy.loginfo("="*60)
    
    def apply_denoise(self, img):
        """应用非局部均值去噪"""
        if len(img.shape) == 2:  # 灰度图
            return cv2.fastNlMeansDenoising(
                img, None,
                h=self.denoise_h,
                templateWindowSize=self.denoise_template_size,
                searchWindowSize=self.denoise_search_size
            )
        else:  # 彩色图
            return cv2.fastNlMeansDenoisingColored(
                img, None,
                h=self.denoise_h,
                hColor=self.denoise_h,
                templateWindowSize=self.denoise_template_size,
                searchWindowSize=self.denoise_search_size
            )
    
    def apply_clahe(self, img):
        """应用CLAHE对比度增强"""
        if len(img.shape) == 2:  # 灰度图
            return self.clahe.apply(img)
        else:  # 彩色图 - 转换到LAB空间
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l = self.clahe.apply(l)
            lab = cv2.merge([l, a, b])
            return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    
    def apply_bilateral_filter(self, img):
        """应用双边滤波"""
        return cv2.bilateralFilter(
            img,
            self.bilateral_d,
            self.bilateral_sigma_color,
            self.bilateral_sigma_space
        )
    
    def apply_sharpen(self, img):
        """应用锐化（Unsharp Mask）"""
        blurred = cv2.GaussianBlur(img, (5, 5), 1.0)
        sharpened = cv2.addWeighted(img, 1.0 + self.sharpen_amount, blurred, -self.sharpen_amount, 0)
        return sharpened
    
    def process_image(self, img):
        """处理单张图像"""
        # 注意：去噪比较耗时，根据实际情况调整参数或禁用
        if self.enable_denoise:
            img = self.apply_denoise(img)
        
        # 双边滤波（可选，与去噪二选一或组合使用）
        if self.enable_bilateral:
            img = self.apply_bilateral_filter(img)
        
        # CLAHE增强对比度（推荐）
        if self.enable_clahe:
            img = self.apply_clahe(img)
        
        # 锐化（可选）
        if self.enable_sharpen:
            img = self.apply_sharpen(img)
        
        return img
    
    def image_callback(self, msg):
        """图像消息回调"""
        try:
            # 转换ROS图像消息为OpenCV格式
            if msg.encoding == 'mono8' or msg.encoding == '8UC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            elif msg.encoding == 'bgr8' or msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                # 尝试自动转换
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 处理图像
            processed = self.process_image(cv_image)
            
            # 转换回ROS消息并发布
            if len(processed.shape) == 2:
                encoding = 'mono8'
            else:
                encoding = 'bgr8'
            
            out_msg = self.bridge.cv2_to_imgmsg(processed, encoding=encoding)
            out_msg.header = msg.header  # 保持时间戳和frame_id
            
            self.image_pub.publish(out_msg)
            
            # 更新统计
            with self.lock:
                self.frame_count += 1
                current_time = rospy.Time.now()
                elapsed = (current_time - self.last_time).to_sec()
                if elapsed >= 5.0:  # 每5秒打印一次
                    fps = self.frame_count / elapsed
                    rospy.loginfo(f"处理帧率: {fps:.2f} Hz | 已处理: {self.frame_count} 帧")
                    self.last_time = current_time
                    self.frame_count = 0
        
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {e}")
    
    def run(self):
        """运行节点"""
        rospy.spin()


def main():
    """主函数"""
    try:
        preprocessor = ImagePreprocessor()
        preprocessor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("图像预处理节点已关闭")
    except Exception as e:
        rospy.logerr(f"节点异常退出: {e}")


if __name__ == '__main__':
    main()
