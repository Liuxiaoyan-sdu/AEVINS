#!/bin/bash
# 图像预处理节点测试脚本

echo "=========================================="
echo "图像预处理节点测试"
echo "=========================================="
echo ""

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS环境未配置，请先运行："
    echo "   source /opt/ros/<your-distro>/setup.bash"
    echo "   source ~/aevins_ws/devel/setup.bash"
    exit 1
fi

echo "✅ ROS环境: $ROS_DISTRO"
echo ""

# 检查节点文件
NODE_FILE="$HOME/aevins_ws/src/tools/scripts/image_preprocessor_node.py"
if [ ! -f "$NODE_FILE" ]; then
    echo "❌ 节点文件不存在: $NODE_FILE"
    exit 1
fi

if [ ! -x "$NODE_FILE" ]; then
    echo "⚠️  节点没有可执行权限，正在添加..."
    chmod +x "$NODE_FILE"
fi

echo "✅ 节点文件: $NODE_FILE"
echo ""

# 检查launch文件
LAUNCH_FILE="$HOME/aevins_ws/src/tools/launch/image_preprocessor.launch"
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "❌ Launch文件不存在: $LAUNCH_FILE"
    exit 1
fi

echo "✅ Launch文件: $LAUNCH_FILE"
echo ""

# 显示使用说明
echo "=========================================="
echo "使用方法："
echo "=========================================="
echo ""
echo "1. 启动图像源（rosbag或相机）："
echo "   rosbag play your_data.bag"
echo ""
echo "2. 在另一个终端启动预处理节点："
echo "   roslaunch tools image_preprocessor.launch"
echo ""
echo "3. 查看处理效果："
echo "   rqt_image_view"
echo "   （选择话题：/cam0/image_preprocessed）"
echo ""
echo "4. 自定义参数示例："
echo "   roslaunch tools image_preprocessor.launch \\"
echo "       input_topic:=/camera/image_raw \\"
echo "       output_topic:=/camera/image_enhanced \\"
echo "       denoise_h:=12 \\"
echo "       clahe_clip_limit:=4.0"
echo ""
echo "=========================================="
echo "快速测试（如果你有图像话题）："
echo "=========================================="
echo ""

read -p "是否现在启动预处理节点？(y/n) " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "正在启动节点..."
    echo "按 Ctrl+C 停止"
    echo ""
    roslaunch tools image_preprocessor.launch
else
    echo ""
    echo "测试脚本完成。手动启动请运行："
    echo "  roslaunch tools image_preprocessor.launch"
    echo ""
fi
