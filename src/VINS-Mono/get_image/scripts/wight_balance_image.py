#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Hard-coded parameters
        self.contrast = 0.8  # Contrast adjustment (1.0 is original, >1.0 increases, <1.0 decreases)
        self.exposure = -9.5  # Exposure adjustment (0.0 is original, positive increases, negative decreases)
        
        # Subscribers and Publishers
        self.image_sub = rospy.Subscriber("/gloab_camera/image", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera/image", Image, queue_size=10)
        
        rospy.loginfo("Image processor node initialized")
        
    def white_balance(self, img):
        try:
            # Split channels
            b, g, r = cv2.split(img)
            
            # Calculate mean values
            b_mean = np.mean(b)
            g_mean = np.mean(g)
            r_mean = np.mean(r)
            
            # Calculate gains
            k = (b_mean + g_mean + r_mean) / 3
            kb = k / b_mean if b_mean != 0 else 1
            kg = k / g_mean if g_mean != 0 else 1
            kr = k / r_mean if r_mean != 0 else 1
            
            # Apply gains
            b = cv2.multiply(b, kb)
            g = cv2.multiply(g, kg)
            r = cv2.multiply(r, kr)
            
            return cv2.merge([b, g, r])
        except Exception as e:
            rospy.logerr("White balance error: {0}".format(e))
            return img
    
    def adjust_contrast_exposure(self, img):
        try:
            # Convert to float for processing
            img_float = img.astype(np.float32)
            
            # Apply contrast
            img_float = img_float * self.contrast
            
            # Apply exposure
            img_float = img_float + self.exposure
            
            # Clip values and convert back to uint8
            img_float = np.clip(img_float, 0, 255)
            return img_float.astype(np.uint8)
        except Exception as e:
            rospy.logerr("Contrast/exposure adjustment error: {0}".format(e))
            return img
    
    def image_callback(self, data):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Apply white balance
            wb_image = self.white_balance(cv_image)
            
            # Adjust contrast and exposure
            processed_image = self.adjust_contrast_exposure(wb_image)
            
            # Convert back to ROS image, preserving original header
            img_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            img_msg.header = data.header  # Ensure timestamp is unchanged
            
            # Publish processed image
            self.image_pub.publish(img_msg)
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        except Exception as e:
            rospy.logerr("Image processing error: {0}".format(e))

def main():
    rospy.init_node('image_processor', anonymous=True)
    try:
        processor = ImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down image processor node")
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image processor node")

if __name__ == '__main__':
    main()