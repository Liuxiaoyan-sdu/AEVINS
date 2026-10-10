#!/usr/bin/env python3
"""
image_darkener_node.py

ROS 节点：订阅图像话题，对图像进行降曝光/压暗处理后重新发布。

支持两种压暗方式（可通过参数选择）：
  1. gamma  (默认) — Gamma 校正，gamma > 1.0 压暗画面，典型值 1.5~3.0
  2. scale          — 线性乘法，scale 在 0.0~1.0 之间，值越小越暗

支持两种输入话题类型：
  - sensor_msgs/Image        （raw，直接订阅话题名）
  - sensor_msgs/CompressedImage（压缩图，话题名须以 /compressed 结尾，
                                 或将 ~input_compressed 设为 true）

ROS 参数（均可在 launch 中设置）：
  ~input_topic        (str,   默认 /dvs/image_raw/compressed) 输入图像话题
  ~input_compressed   (bool,  默认 true)  输入是否为压缩图像
  ~output_topic       (str,   默认 /dvs/image_raw)            输出图像话题（raw）
  ~method             (str,   默认 gamma)  压暗方式：gamma | scale
  ~gamma              (float, 默认 2.0)   Gamma 值，>1 变暗，<1 变亮
  ~scale              (float, 默认 0.5)   线性缩放系数，0.0~1.0
  ~queue_size         (int,   默认 1)     订阅/发布队列长度

典型用法（替代 image_transport republish）：
  订阅 /dvs/image_raw/compressed（压缩），解压后压暗，
  输出 raw sensor_msgs/Image 到 /dvs/image_raw，下游节点无需感知压缩。
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class ImageDarkener:
    """图像压暗 ROS 节点，支持压缩与非压缩输入"""

    def __init__(self):
        rospy.init_node('image_darkener', anonymous=False)

        # ── 读取参数 ──────────────────────────────────────────────────────
        self.input_topic      = rospy.get_param('~input_topic',
                                                '/dvs/image_raw/compressed')
        self.input_compressed = rospy.get_param('~input_compressed', True)
        self.output_topic     = rospy.get_param('~output_topic',
                                                '/dvs/image_raw')
        self.method           = rospy.get_param('~method', 'gamma')
        self.gamma            = float(rospy.get_param('~gamma', 2.0))
        self.scale            = float(rospy.get_param('~scale', 0.5))
        self.queue_size       = int(rospy.get_param('~queue_size', 1))

        # 若话题名本身以 /compressed 结尾，自动识别为压缩输入
        if self.input_topic.endswith('/compressed'):
            self.input_compressed = True

        # 参数校验 ────────────────────────────────────────────────────────
        if self.method not in ('gamma', 'scale'):
            rospy.logwarn(
                f"[image_darkener] 未知 method='{self.method}'，"
                "自动回退为 'gamma'")
            self.method = 'gamma'

        if self.gamma <= 0:
            rospy.logwarn("[image_darkener] gamma 必须 > 0，已重置为 2.0")
            self.gamma = 2.0

        self.scale = float(np.clip(self.scale, 0.0, 1.0))

        # ── 预计算 Gamma LUT（uint8 快速查表）────────────────────────────
        self._build_gamma_lut()

        # ── cv_bridge ────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── 发布器（始终输出 raw Image）──────────────────────────────────
        self.pub = rospy.Publisher(
            self.output_topic, Image, queue_size=self.queue_size)

        # ── 订阅器（根据输入类型选择消息类型）───────────────────────────
        if self.input_compressed:
            self.sub = rospy.Subscriber(
                self.input_topic, CompressedImage,
                self.callback_compressed,
                queue_size=self.queue_size,
                buff_size=2 ** 24)
            input_type = 'CompressedImage'
        else:
            self.sub = rospy.Subscriber(
                self.input_topic, Image,
                self.callback_raw,
                queue_size=self.queue_size,
                buff_size=2 ** 24)
            input_type = 'Image (raw)'

        rospy.loginfo(
            f"[image_darkener] 启动\n"
            f"  输入  : {self.input_topic}  [{input_type}]\n"
            f"  输出  : {self.output_topic}  [Image raw]\n"
            f"  方式  : {self.method}"
            + (f"  (gamma={self.gamma})" if self.method == 'gamma'
               else f"  (scale={self.scale})")
        )

    # ─────────────────────────────────────────────────────────────────────

    def _build_gamma_lut(self):
        """预计算 [0, 255] 的 Gamma 查找表，加速逐帧处理。
        output = (input/255)^gamma * 255
        gamma > 1 → 图像变暗；gamma < 1 → 图像变亮
        """
        table = np.array(
            [((i / 255.0) ** self.gamma) * 255 for i in range(256)],
            dtype=np.uint8
        )
        self.gamma_lut = table

    # ─────────────────────────────────────────────────────────────────────

    def _darken_gamma(self, img: np.ndarray) -> np.ndarray:
        """Gamma 压暗：output = (input/255)^gamma * 255，gamma>1 时图像变暗。"""
        if img.dtype != np.uint8:
            img = np.clip(img, 0, 255).astype(np.uint8)
        return cv2.LUT(img, self.gamma_lut)

    def _darken_scale(self, img: np.ndarray) -> np.ndarray:
        """线性缩放压暗：output = input * scale，scale<1 时图像变暗。"""
        return cv2.convertScaleAbs(img, alpha=self.scale, beta=0)

    def _darken(self, img: np.ndarray) -> np.ndarray:
        if self.method == 'gamma':
            return self._darken_gamma(img)
        else:
            return self._darken_scale(img)

    # ─────────────────────────────────────────────────────────────────────

    def _cv_img_to_ros(self, cv_img: np.ndarray, header, src_encoding: str) -> Image:
        """将 OpenCV 图像转换为 ROS Image 消息，保留时间戳与 frame_id。"""
        # 输出 encoding 跟随输入（灰度保持 mono8，彩色保持 bgr8）
        out_encoding = src_encoding if src_encoding in ('mono8', '8UC1', 'bgr8', 'rgb8') else 'bgr8'
        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding=out_encoding)
        except CvBridgeError:
            # 如果 encoding 不匹配则回退
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='passthrough')
        out_msg.header = header
        return out_msg

    # ─────────────────────────────────────────────────────────────────────

    def callback_compressed(self, msg: CompressedImage):
        """处理 sensor_msgs/CompressedImage 输入。"""
        # 用 numpy + imdecode 解压，避免 cv_bridge 对压缩图的兼容性问题
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        if cv_img is None:
            rospy.logerr("[image_darkener] imdecode 失败，跳过此帧")
            return

        darkened = self._darken(cv_img)

        # 判断输出 encoding
        if len(darkened.shape) == 2 or (len(darkened.shape) == 3 and darkened.shape[2] == 1):
            enc = 'mono8'
        else:
            enc = 'bgr8'

        out_msg = self._cv_img_to_ros(darkened, msg.header, enc)
        self.pub.publish(out_msg)

    # ─────────────────────────────────────────────────────────────────────

    def callback_raw(self, msg: Image):
        """处理 sensor_msgs/Image（raw）输入。"""
        try:
            if msg.encoding in ('mono8', '8UC1'):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            elif msg.encoding in ('bgr8', 'rgb8', 'bgra8', 'rgba8'):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(f"[image_darkener] cv_bridge 转换失败: {e}")
            return

        darkened = self._darken(cv_img)
        out_msg = self._cv_img_to_ros(darkened, msg.header, msg.encoding)
        self.pub.publish(out_msg)

    # ─────────────────────────────────────────────────────────────────────

    def spin(self):
        rospy.spin()


# ─────────────────────────────────────────────────────────────────────────────

def main():
    darkener = ImageDarkener()
    darkener.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
