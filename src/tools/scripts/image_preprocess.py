#!/usr/bin/env python3
"""
image_preprocess.py

批量图像预处理：去噪（NLMeans），自适应直方图均衡（CLAHE），可选锐化。
将处理后的图像保存到输出目录（默认：./processed）。

用法示例:
  cd /path/to/that/folder
  python3 image_preprocess.py --in-dir . --out-dir processed --ext png --denoise --clahe --sharpen

依赖:
  pip install opencv-python numpy

作者: 自动生成
"""
import os
import sys
import argparse
import glob
import logging
from typing import Tuple

try:
    import cv2
    import numpy as np
except Exception as e:
    print("缺少依赖：请先安装 opencv-python 和 numpy，例如: pip install opencv-python numpy")
    raise

# 可选的 ROS 发布支持
_HAS_ROS = False
try:
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image as RosImage
    _HAS_ROS = True
except Exception:
    # 在没有 ROS 环境时脚本依然可运行（只不过无法发布）
    _HAS_ROS = False

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")


def ensure_dir(path: str) -> None:
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)


def read_image(path: str):
    img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    # cv2.imread may fail on paths with non-ascii on some systems; use imdecode as more robust
    return img


def save_image(path: str, img) -> bool:
    # use imencode to preserve unicode paths and formats
    ext = os.path.splitext(path)[1]
    if ext == "":
        ext = ".png"
    success, enc = cv2.imencode(ext, img)
    if not success:
        return False
    try:
        with open(path, "wb") as f:
            enc.tofile(f)
        return True
    except Exception:
        return False


def apply_clahe_color(img, clip_limit=2.0, tile_grid_size=(8, 8)):
    # 对彩色图像使用 CLAHE：转换到 LAB 空间，对 L 通道做 CLAHE
    if img is None:
        return img
    if len(img.shape) == 2 or img.shape[2] == 1:
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
        return clahe.apply(img)

    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    l2 = clahe.apply(l)
    lab2 = cv2.merge((l2, a, b))
    return cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)


def denoise_nlmeans(img, h=10, hColor=10, templateWindowSize=7, searchWindowSize=21):
    if img is None:
        return img
    # 对灰度和彩图分别处理
    if len(img.shape) == 2 or (img.ndim == 2):
        return cv2.fastNlMeansDenoising(img, None, h, templateWindowSize, searchWindowSize)
    else:
        return cv2.fastNlMeansDenoisingColored(img, None, h, hColor, templateWindowSize, searchWindowSize)


def unsharp_mask(img, kernel_size=(5, 5), sigma=1.0, amount=1.0, threshold=0):
    # 简单的反锐化遮罩
    if img is None:
        return img
    blurred = cv2.GaussianBlur(img, kernel_size, sigma)
    sharpened = cv2.addWeighted(img, 1 + amount, blurred, -amount, 0)
    if threshold > 0:
        low_contrast_mask = np.absolute(img - blurred) < threshold
        np.copyto(sharpened, img, where=low_contrast_mask)
    return sharpened


def process_image(img, do_denoise=True, denoise_params=None, do_clahe=True, clahe_params=None, do_sharpen=False, sharpen_params=None):
    out = img.copy() if img is not None else None

    if do_denoise:
        params = denoise_params or {}
        out = denoise_nlmeans(out, **params)

    if do_clahe:
        params = clahe_params or {}
        out = apply_clahe_color(out, **params)

    if do_sharpen:
        params = sharpen_params or {}
        out = unsharp_mask(out, **params)

    return out


def gather_images(input_dir: str, ext: str):
    p = os.path.join(input_dir, f"*.{ext}")
    files = sorted(glob.glob(p))
    return files


def parse_args():
    p = argparse.ArgumentParser(description="批量图像预处理（去噪 + CLAHE + 可选锐化）")
    p.add_argument("--in-dir", default=".", help="输入目录（默认: 当前目录）")
    p.add_argument("--out-dir", default="processed", help="输出目录（默认: processed）")
    p.add_argument("--ext", default="png", help="要处理的图像扩展名（不带点），例如 png jpg bmp）")
    p.add_argument("--denoise", action="store_true", help="启用 NLMeans 去噪")
    p.add_argument("--denoise-h", type=float, default=10.0, help="去噪强度 h")
    p.add_argument("--denoise-hcolor", type=float, default=10.0, help="去噪强度 hColor（彩色图像）")
    p.add_argument("--clahe", action="store_true", help="启用 CLAHE 自适应直方图均衡")
    p.add_argument("--clahe-clip", type=float, default=2.0, help="CLAHE clipLimit")
    p.add_argument("--clahe-grid", type=int, default=8, help="CLAHE tileGridSize（单边）")
    p.add_argument("--sharpen", action="store_true", help="启用反锐化遮罩（Unsharp Mask）")
    p.add_argument("--sharpen-amount", type=float, default=0.8, help="锐化强度（amount）")
    p.add_argument("--overwrite", action="store_true", help="覆盖输出目录中已存在的文件")
    p.add_argument("--verbose", action="store_true", help="打印更多日志")
    # ROS 发布相关
    p.add_argument("--ros-publish", action="store_true", help="启用 ROS 发布，需在 ROS 环境下运行")
    p.add_argument("--ros-topic", type=str, default="/processed_image", help="ROS 话题名（默认: /processed_image）")
    p.add_argument("--ros-rate", type=float, default=5.0, help="发布速率 Hz（默认: 5.0）")
    p.add_argument("--no-save", action="store_true", help="只发布到 ROS，不保存到磁盘")
    return p.parse_args()


def main():
    args = parse_args()
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    in_dir = os.path.abspath(args.in_dir)
    out_dir = os.path.join(in_dir, args.out_dir)
    ensure_dir(out_dir)

    files = gather_images(in_dir, args.ext)
    if len(files) == 0:
        logging.warning(f"在 {in_dir} 中未找到扩展名为 {args.ext} 的图片。使用通配符搜索。")
        # 支持多后缀
        exts = ["png", "jpg", "jpeg", "bmp", "tiff"]
        files = []
        for e in exts:
            files.extend(gather_images(in_dir, e))
        files = sorted(files)
        if len(files) == 0:
            logging.error("仍未找到任何图片。退出。")
            sys.exit(1)

    logging.info(f"发现 {len(files)} 张待处理图片，输出目录: {out_dir}")

    denoise_params = {"h": args.denoise_h, "hColor": args.denoise_hcolor, "templateWindowSize": 7, "searchWindowSize": 21}
    clahe_params = {"clip_limit": args.clahe_clip, "tile_grid_size": (args.clahe_grid, args.clahe_grid)}
    sharpen_params = {"kernel_size": (5, 5), "sigma": 1.0, "amount": args.sharpen_amount, "threshold": 0}

    # ROS 初始化（如需发布）
    bridge = None
    pub = None
    rate = None
    if args.ros_publish:
        if not _HAS_ROS:
            logging.error("启用 --ros-publish，但未检测到 rospy / cv_bridge，无法发布。请在 ROS 环境中安装并使用。")
            sys.exit(1)
        # 初始化 ROS 节点
        rospy.init_node('image_preprocess_publisher', anonymous=True)
        bridge = CvBridge()
        pub = rospy.Publisher(args.ros_topic, RosImage, queue_size=10)
        # rate 可以是 float Hz
        try:
            rate = rospy.Rate(args.ros_rate)
        except Exception:
            rate = None

    for idx, fpath in enumerate(files, 1):
        fname = os.path.basename(fpath)
        out_path = os.path.join(out_dir, fname)
        if os.path.exists(out_path) and not args.overwrite:
            logging.info(f"[{idx}/{len(files)}] 跳过已存在: {fname}")
            continue

        logging.info(f"[{idx}/{len(files)}] 读取: {fname}")
        img = read_image(fpath)
        if img is None:
            logging.warning(f"无法读取图片: {fpath}，跳过。")
            continue

        processed = process_image(img,
                                  do_denoise=args.denoise,
                                  denoise_params=denoise_params,
                                  do_clahe=args.clahe,
                                  clahe_params=clahe_params,
                                  do_sharpen=args.sharpen,
                                  sharpen_params=sharpen_params)

        # 保存（除非用户指定 --no-save）
        if not args.no_save:
            ok = save_image(out_path, processed)
            if not ok:
                logging.error(f"保存失败: {out_path}")
            else:
                logging.debug(f"保存: {out_path}")

        # 发布到 ROS 话题（如果启用）
        if args.ros_publish:
            try:
                # 选择合适的 encoding
                if processed is None:
                    logging.warning(f"处理结果为空，跳过发布: {fname}")
                else:
                    if processed.ndim == 2 or (processed.ndim == 3 and processed.shape[2] == 1):
                        encoding = 'mono8'
                    else:
                        # OpenCV uses BGR ordering
                        encoding = 'bgr8'
                    msg = bridge.cv2_to_imgmsg(processed, encoding=encoding)
                    msg.header.stamp = rospy.Time.now()
                    pub.publish(msg)
                    logging.info(f"已发布到 {args.ros_topic}: {fname}")
            except CvBridgeError as e:
                logging.error(f"CvBridge 错误: {e}")
            except Exception as e:
                logging.error(f"发布时出错: {e}")
            # 控制发布频率
            if rate is not None:
                try:
                    rate.sleep()
                except Exception:
                    # 可能在非 ROS 主线程或 node 已被关闭
                    pass

    logging.info("处理完成。")


if __name__ == '__main__':
    main()
