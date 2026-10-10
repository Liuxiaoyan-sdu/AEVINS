# AEVINS

AEVINS 是基于 VINS-Mono 的 ROS1 多传感器状态估计工程，包含图像、事件相机、IMU 和 DVL 相关的前端、后端与辅助工具。本仓库只保存源码、配置和启动文件；`build/`、`devel/`、数据集及运行结果不会纳入版本控制。

## 运行环境

当前工程在以下环境中使用：

- Ubuntu 20.04
- ROS Noetic（ROS1）
- OpenCV、Eigen3、Ceres Solver 1.14
- PCL、TBB、Boost

建议使用相同环境。Ceres 2.x 与原版 VINS-Mono 的部分接口不兼容，Ubuntu 20.04 官方源中的 `libceres-dev` 为 1.14，适合本工程。

## 安装

### 1. 安装 ROS Noetic

按照 [ROS Noetic 官方安装说明](http://wiki.ros.org/noetic/Installation/Ubuntu) 完成 ROS 安装，然后初始化 rosdep：

```bash
sudo rosdep init       # 已初始化过则跳过
rosdep update
```

### 2. 获取源码

```bash
git clone https://github.com/Liuxiaoyan-sdu/AEVINS.git ~/aevins_ws
cd ~/aevins_ws
```

仓库本身就是 catkin 工作空间，所有 ROS 包均位于 `src/` 下，不要再套一层 `catkin_ws/src`。

### 3. 安装依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libceres-dev libeigen3-dev libopencv-dev libpcl-dev libtbb-dev \
  libboost-all-dev libcaer-dev \
  python3-catkin-tools python3-rosdep \
  ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-message-filters ros-noetic-tf ros-noetic-tf-conversions \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-laser-geometry ros-noetic-rviz

rosdep install --from-paths src --ignore-src -r -y
```

若不使用 DAVIS/DVXplorer 事件相机驱动，可在缺少 `libcaer` 时忽略对应驱动包，或安装系统提供的 `libcaer-dev` 后再完整编译。

### 4. 编译

```bash
cd ~/aevins_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

如需每次打开终端自动加载工作空间：

```bash
echo 'source ~/aevins_ws/devel/setup.bash' >> ~/.bashrc
```

## 运行

先根据传感器话题、标定参数和数据路径选择或修改 `src/VINS-Mono/config/euroc/` 下的 YAML，再启动对应 launch 文件。例如：

```bash
source ~/aevins_ws/devel/setup.bash
roslaunch vins_estimator euroc_sea.launch
```

另开终端启动 RViz：

```bash
source ~/aevins_ws/devel/setup.bash
roslaunch vins_estimator vins_rviz.launch
```

最后播放数据包：

```bash
rosbag play /path/to/dataset.bag
```

不同实验配置可使用 `euroc_event.launch`、`euroc_sea2.launch`、`euroc_sea3.launch`、`euroc_pool_*.launch` 等文件。运行前请重点检查配置中的图像、事件、IMU、DVL 话题名，以及相机内外参和时间偏移。

## 目录说明

- `src/VINS-Mono/`：VINS 估计器、图像/事件特征跟踪、回环与配置
- `src/rpg_dvs_ros/`：事件相机 ROS 消息及驱动
- `src/tools/`：IMU/DVL 与图像预处理辅助节点
- `src/catkin_simple/`：部分事件相机包需要的 catkin 辅助模块
- `src/cognifli_*`：仿真及硬件相关 ROS 包

数据集、rosbag、编译目录和运行结果请保存在仓库之外，或放入已被 `.gitignore` 排除的目录。

## 许可

VINS-Mono 原始代码采用 GPLv3，详见 `src/VINS-Mono/LICENCE`。各第三方组件的许可信息以其目录内文件为准。
