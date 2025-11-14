# 图像预处理节点 (Image Preprocessor Node)

## 功能说明

这个ROS节点专门用于解决VINS-Mono视觉SLAM中**图像对比度低、噪点多导致特征匹配错误**的问题。

### 主要功能
- ✅ **去噪处理**：使用非局部均值去噪（fastNlMeansDenoising）去除图像噪点
- ✅ **对比度增强**：使用CLAHE（自适应直方图均衡）提升图像对比度
- ✅ **实时处理**：订阅ROS图像话题，实时处理并发布
- ✅ **参数可调**：支持通过launch文件或参数服务器调整所有处理参数
- ✅ **可选滤波**：支持双边滤波和锐化（根据需要开启）

### 解决的问题
1. **噪点匹配错误**：去噪后减少噪点被误认为特征点
2. **对比度不足**：CLAHE增强后提升结构点的可检测性
3. **特征提取困难**：处理后的图像特征更加明显

---

## 依赖

- ROS (Kinetic/Melodic/Noetic)
- Python 3
- OpenCV (python3-opencv)
- cv_bridge
- sensor_msgs

安装依赖：
```bash
# Ubuntu 18.04/20.04
sudo apt-get install python3-opencv ros-$ROS_DISTRO-cv-bridge

# 或使用pip
pip3 install opencv-python
```

---

## 文件结构

```
src/tools/
├── scripts/
│   ├── image_preprocessor_node.py   # 主节点
│   └── image_preprocess.py          # 离线批处理脚本（可选）
└── launch/
    └── image_preprocessor.launch    # 启动文件
```

---

## 快速开始

### 1. 编译工作区（如果需要）

```bash
cd ~/vins_mono_ws
catkin_make
source devel/setup.bash
```

### 2. 基本使用

**方法一：使用launch文件（推荐）**

```bash
# 默认参数启动（输入：/cam0/image_raw，输出：/cam0/image_preprocessed）
roslaunch tools image_preprocessor.launch

# 自定义输入输出话题
roslaunch tools image_preprocessor.launch \
    input_topic:=/camera/image_raw \
    output_topic:=/camera/image_enhanced

# 调整去噪强度（h值越大，去噪越强，但可能过度模糊）
roslaunch tools image_preprocessor.launch denoise_h:=15

# 调整CLAHE对比度增强强度
roslaunch tools image_preprocessor.launch clahe_clip_limit:=4.0

# 启用锐化（增强边缘）
roslaunch tools image_preprocessor.launch enable_sharpen:=true sharpen_amount:=0.8
```

**方法二：直接运行节点**

```bash
rosrun tools image_preprocessor_node.py \
    _input_topic:=/cam0/image_raw \
    _output_topic:=/cam0/image_preprocessed
```

### 3. 配合VINS-Mono使用

**修改VINS-Mono的launch文件**，将原来的图像话题改为预处理后的话题：

```xml
<!-- 原来 -->
<param name="image_topic" value="/cam0/image_raw" />

<!-- 改为 -->
<param name="image_topic" value="/cam0/image_preprocessed" />
```

**完整启动流程：**

```bash
# 终端1：启动图像源（如rosbag或相机驱动）
rosbag play your_data.bag

# 终端2：启动图像预处理节点
roslaunch tools image_preprocessor.launch

# 终端3：启动VINS-Mono（订阅处理后的话题）
roslaunch vins_estimator vins_rviz.launch
```

---

## 参数说明

### 核心参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `input_topic` | string | `/cam0/image_raw` | 输入图像话题 |
| `output_topic` | string | `/cam0/image_preprocessed` | 输出图像话题 |

### 去噪参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_denoise` | bool | `true` | 是否启用去噪 |
| `denoise_h` | float | `10` | 去噪强度（3-15，越大越强）|
| `denoise_template_size` | int | `7` | 模板窗口大小（7或9） |
| `denoise_search_size` | int | `21` | 搜索窗口大小（15-25） |

**调参建议：**
- 噪点较多：`denoise_h=12-15`
- 噪点中等：`denoise_h=8-10`（默认）
- 噪点较少：`denoise_h=5-8` 或禁用

### CLAHE参数（对比度增强）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_clahe` | bool | `true` | 是否启用CLAHE |
| `clahe_clip_limit` | float | `3.0` | 对比度限制（2.0-4.0） |
| `clahe_tile_size` | int | `8` | 网格大小（8或16） |

**调参建议：**
- 对比度极低：`clahe_clip_limit=4.0`
- 对比度一般：`clahe_clip_limit=2.5-3.0`（默认）
- 对比度尚可：`clahe_clip_limit=2.0` 或禁用

### 可选功能

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_bilateral` | bool | `false` | 双边滤波（保边去噪） |
| `enable_sharpen` | bool | `false` | 锐化增强边缘 |
| `sharpen_amount` | float | `0.5` | 锐化强度（0.3-1.0） |

---

## 效果验证

### 1. 实时查看处理效果

**使用rqt_image_view对比原图和处理后的图像：**

```bash
# 安装工具（如果没有）
sudo apt-get install ros-$ROS_DISTRO-rqt-image-view

# 启动查看器
rqt_image_view
```

在界面中：
- 左侧下拉菜单选择 `/cam0/image_raw` 查看原图
- 再打开一个rqt_image_view，选择 `/cam0/image_preprocessed` 查看处理后

### 2. 检查话题

```bash
# 查看所有图像话题
rostopic list | grep image

# 查看处理后话题的发布频率
rostopic hz /cam0/image_preprocessed

# 查看话题信息
rostopic info /cam0/image_preprocessed
```

### 3. 录制对比数据

```bash
# 录制原图和处理后的图
rosbag record /cam0/image_raw /cam0/image_preprocessed -O comparison.bag
```

---

## 性能优化

### 实时性考虑

去噪处理（NLMeans）比较耗时，如果帧率下降明显：

**方案1：降低去噪强度**
```bash
roslaunch tools image_preprocessor.launch denoise_h:=8 denoise_search_size:=15
```

**方案2：只使用CLAHE**
```bash
roslaunch tools image_preprocessor.launch enable_denoise:=false
```

**方案3：使用双边滤波代替NLMeans**
```bash
roslaunch tools image_preprocessor.launch \
    enable_denoise:=false \
    enable_bilateral:=true
```

### 预期处理速度

- **仅CLAHE**：~30-50 FPS（640x480）
- **CLAHE + NLMeans**：~5-15 FPS（640x480）
- **CLAHE + 双边滤波**：~20-30 FPS（640x480）

---

## 常见问题

### Q1: 节点启动后没有输出图像

**检查步骤：**
```bash
# 1. 确认输入话题是否有数据
rostopic hz /cam0/image_raw

# 2. 检查节点是否运行
rosnode list | grep preprocessor

# 3. 查看节点日志
rosnode info /image_preprocessor
```

### Q2: 处理速度太慢

- 禁用去噪：`enable_denoise:=false`
- 或降低去噪参数：`denoise_h:=8 denoise_search_size:=15`
- 减小图像分辨率（在图像源处处理）

### Q3: 处理后效果不明显

- 增加CLAHE强度：`clahe_clip_limit:=4.0`
- 增加去噪强度：`denoise_h:=12`
- 启用锐化：`enable_sharpen:=true`

### Q4: 图像过度模糊

- 降低去噪强度：`denoise_h:=5`
- 禁用双边滤波：`enable_bilateral:=false`
- 或完全禁用去噪，只用CLAHE

---

## 离线批处理脚本

如果需要对已有图像进行批处理（不使用ROS），可以使用：

```bash
cd ~/vins_mono_ws/src/tools/scripts
python3 image_preprocess.py --in-dir ./images --out-dir ./processed \
    --denoise --clahe --overwrite
```

详见 `image_preprocess.py --help`

---

## 技术细节

### 处理流程

```
原始图像
   ↓
[1] 去噪（fastNlMeansDenoising） ← 可选
   ↓
[2] 双边滤波（bilateralFilter） ← 可选
   ↓
[3] CLAHE对比度增强 ← 推荐
   ↓
[4] 锐化（Unsharp Mask） ← 可选
   ↓
处理后图像 → 发布到ROS话题
```

### 算法说明

- **NLMeans去噪**：保留纹理细节的同时去除噪点，适合强噪声
- **CLAHE**：自适应直方图均衡，局部增强对比度，避免全局过曝
- **双边滤波**：保边去噪，速度快于NLMeans
- **锐化**：增强边缘，但可能放大噪点

---

## 贡献与反馈

如有问题或建议，请提交Issue或PR。

**推荐参数组合：**

1. **均衡模式**（推荐）：
   ```bash
   enable_denoise=true, denoise_h=10
   enable_clahe=true, clahe_clip_limit=3.0
   ```

2. **快速模式**（高帧率优先）：
   ```bash
   enable_denoise=false
   enable_clahe=true, clahe_clip_limit=3.0
   ```

3. **高质量模式**（精度优先）：
   ```bash
   enable_denoise=true, denoise_h=12
   enable_clahe=true, clahe_clip_limit=4.0
   enable_sharpen=true, sharpen_amount=0.6
   ```

---

## License

遵循VINS-Mono工作区的许可证。
