# 水下SLAM特征匹配参数调优指南

## 问题描述
水下环境存在以下挑战：
1. **虚无状态**：水体中悬浮物、光线折射导致图像模糊
2. **噪点匹配**：悬浮颗粒、气泡等产生大量噪点
3. **结构点提取困难**：真实的场景特征被噪点淹没

## 核心参数优化策略

### 1. 特征点数量与分布 (canghewan.yaml)

#### `max_cnt: 80` (原值: 100)
**作用**: 控制每帧最大特征点数量  
**调优建议**:
- 降低特征点数量，聚焦于更稳定、质量更高的特征
- 水下环境：**60-100** (建议80)
- 过多特征点会增加噪点匹配概率
- 过少会导致跟踪失败

**调试步骤**:
```yaml
# 保守策略 (噪点很多时)
max_cnt: 60

# 平衡策略 (推荐)
max_cnt: 80

# 激进策略 (水质较好时)
max_cnt: 100
```

#### `min_dist: 15` (原值: 10)
**作用**: 特征点之间的最小间距（像素）  
**调优建议**:
- 增大间距可防止特征点聚集在噪点密集区域
- 水下环境：**15-25** (建议15-20)
- 过大会导致大场景特征点不足
- 过小会导致噪点区域特征点过密

**调试步骤**:
```yaml
# 噪点严重 (悬浮物多、能见度低)
min_dist: 20

# 中等环境 (推荐)
min_dist: 15

# 清水环境
min_dist: 12
```

---

### 2. RANSAC外点剔除

#### `F_threshold: 1.5` (原值: 1.0)
**作用**: Fundamental Matrix RANSAC的阈值（像素）  
**调优建议**:
- 控制geometric verification的严格程度
- 水下环境：**1.2-2.0** (建议1.5)
- 值越大，保留的匹配越多（但可能包含误匹配）
- 值越小，剔除越严格（可能丢失正确匹配）

**调试步骤**:
```yaml
# 严格剔除 (噪点匹配特别多时)
F_threshold: 1.2

# 平衡策略 (推荐)
F_threshold: 1.5

# 宽松策略 (特征点不足时)
F_threshold: 2.0
```

#### `F_threshold_1: 1.0` (原值: 0.7)
**作用**: Event相机的RANSAC阈值  
**调优建议**:
- Event图像通常噪声更大，需要更严格的验证
- 水下环境：**0.8-1.2** (建议1.0)

---

### 3. 图像预处理

#### `equalize: 1` (保持开启)
**作用**: 启用CLAHE自适应直方图均衡化  
**代码实现** (feature_tracker.cpp):
```cpp
cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
clahe->apply(_img, img);
```

**调优建议**:
- **必须开启** (equalize: 1)
- 水下对比度低，CLAHE能有效增强局部对比度

**高级调优** (需修改代码):
如果仍然效果不佳，可以调整CLAHE参数：
```cpp
// 在 feature_tracker.cpp:87 附近修改
// clipLimit: 2.0-4.0 (值越大增强越强，但噪声放大也越多)
// tileGridSize: 小网格(4,4)增强更局部，大网格(16,16)更平滑
cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.5, cv::Size(8, 8));
```

---

### 4. 光流跟踪参数

**代码位置**: feature_tracker.cpp:113
```cpp
cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, 
                         status, err, cv::Size(21, 21), 3);
```

**参数说明**:
- `cv::Size(21, 21)`: 光流窗口大小
- `3`: 金字塔层数

**调优建议** (需修改代码):
```cpp
// 水下噪点多时，增大窗口和金字塔层数
// 窗口大小: 15-31 (奇数)
// 金字塔层数: 3-5
cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, 
                         status, err, cv::Size(25, 25), 4);
```

---

### 5. 特征检测质量阈值

**代码位置**: feature_tracker.cpp:149
```cpp
cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 
                        0.01, MIN_DIST, mask);
```

**参数说明**:
- `0.01`: 角点质量阈值 (Harris/Shi-Tomasi)

**调优建议** (需修改代码):
```cpp
// 提高质量阈值，只选择更显著的角点
// 默认: 0.01
// 水下建议: 0.015-0.03
cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 
                        0.02, MIN_DIST, mask);
```

---

---

## 新增：视觉约束鲁棒核函数（针对虚无区域）

### 问题场景
水下前方虚无（无纹理、悬浮物、能见度极低）时，特征点匹配会产生大量误匹配，导致：
- 重投影误差很大的"外点"参与优化
- 优化结果被错误约束拉偏
- 轨迹发散或漂移

### 解决方案
为视觉约束添加**鲁棒核函数**（Robust Kernel），当重投影误差超过阈值时：
- **降低权重**（软核函数）：Cauchy/Saturated
- **完全剔除**（硬阈值）：误差大的点权重=0，不参与优化

### 配置参数 (canghewan.yaml)

```yaml
# 视觉约束鲁棒核函数参数
visual_loss_threshold: 5.0  # 重投影误差阈值（像素）
use_hard_threshold: 1       # 核函数类型
```

#### `visual_loss_threshold` (重投影误差阈值)
**作用**: 判断特征点是否为外点的阈值（像素）

**调优建议**:
- 正常水下环境：**3.0-5.0**
- 虚无严重（能见度很低）：**5.0-8.0**
- 水质清澈：**2.0-3.0**

**调试步骤**:
```yaml
# 保守策略（严重虚无，大量误匹配）
visual_loss_threshold: 8.0

# 平衡策略（推荐）
visual_loss_threshold: 5.0

# 激进策略（水质好，担心丢失正确匹配）
visual_loss_threshold: 3.0
```

#### `use_hard_threshold` (核函数类型)
**作用**: 选择如何处理超过阈值的误差

**选项说明**:
- **0: Cauchy软核函数**（默认）
  - 对大误差降权但不完全剔除
  - 数学形式: ρ(s) = log(1 + s/c²)
  - 优点: 平滑，对噪声容忍度高
  - 缺点: 极端外点仍有小权重
  
- **1: 硬阈值**（推荐用于虚无区域）
  - 超过阈值的残差权重=0，完全不参与优化
  - 数学形式: ρ(s) = s (if s<c²), ρ(s) = c² (if s≥c²)
  - 优点: 彻底剔除误匹配
  - 缺点: 不连续，可能影响收敛
  
- **2: 饱和损失**（平滑过渡）
  - 在阈值附近平滑降权
  - 数学形式: ρ(s) = 2c²(√(1+s/c²) - 1)
  - 优点: 兼顾鲁棒性和平滑性
  - 缺点: 调参稍复杂

**推荐配置**:
```yaml
# 虚无严重，需要激进剔除
use_hard_threshold: 1
visual_loss_threshold: 6.0

# 中等环境，平滑处理
use_hard_threshold: 2
visual_loss_threshold: 4.0

# 水质较好，保守处理
use_hard_threshold: 0
visual_loss_threshold: 3.0
```

### 验证效果

1. **查看优化残差**:
```bash
# 启动后观察终端输出
# 会打印使用的损失函数类型和阈值
[INFO] Using HardThresholdLoss with threshold: 5.00 pixels
```

2. **对比轨迹精度**:
```bash
# 录制rosbag，分别用不同参数运行，对比轨迹
rosbag record /vins_estimator/odometry /vins_estimator/path -O test_loss_func.bag
```

3. **可视化特征点**:
- 观察 `/feature_tracker/feature_img` 话题
- 误匹配的特征点应该被剔除，不再影响轨迹

### 与特征跟踪参数的配合

**组合策略**（针对虚无区域）:
```yaml
# 特征跟踪层面：减少误提取
max_cnt: 60              # 减少特征点数量
min_dist: 20             # 增大间距
F_threshold: 1.8         # 严格RANSAC

# 优化层面：剔除误匹配
visual_loss_threshold: 6.0  # 大阈值
use_hard_threshold: 1       # 硬剔除
```

---

## 推荐调优流程

### 阶段1: 快速配置 (仅修改YAML，无需重新编译)

1. **备份原配置**:
```bash
cd /home/lxy/vins_mono_ws/src/VINS-Mono/config/euroc
cp canghewan.yaml canghewan.yaml.backup
```

2. **应用保守配置** (噪点严重时):
```yaml
max_cnt: 60
min_dist: 20
F_threshold: 1.8
F_threshold_1: 1.2
equalize: 1
```

3. **运行并观察**:
```bash
cd /home/lxy/vins_mono_ws
source devel/setup.bash
roslaunch vins_estimator euroc_changhewan.launch
# 播放rosbag，观察/feature_tracker/feature_img话题
```

4. **逐步调整**:
   - 如果特征点太少导致跟踪丢失 → 增加max_cnt、降低F_threshold
   - 如果仍然误匹配严重 → 增加min_dist、提高F_threshold

### 阶段2: 代码级优化 (需重新编译)

如果YAML调整后仍不理想，修改代码：

1. **增强CLAHE**:
```bash
vim src/VINS-Mono/feature_tracker/src/feature_tracker.cpp
# 找到第87行，修改：
cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.5, cv::Size(8, 8));
```

2. **提高角点质量**:
```bash
# 找到第149行，修改：
cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 
                        0.02, MIN_DIST, mask);
```

3. **增强光流鲁棒性**:
```bash
# 找到第113行，修改：
cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, 
                         status, err, cv::Size(25, 25), 4);
```

4. **重新编译**:
```bash
cd /home/lxy/vins_mono_ws
catkin_make
source devel/setup.bash
```

### 阶段3: 图像预处理节点 (已创建)

使用独立的预处理节点进一步增强图像：

```bash
# 启动预处理节点
rosrun tools image_preprocessor_node.py \
    --input-topic /dvs/image_raw \
    --output-topic /dvs/image_preprocessed \
    --denoise --clahe --sharpen

# 修改launch文件，让VINS订阅预处理后的话题
# 在euroc_changhewan.launch中修改image1_topic
```

---

## 参数对照表

| 参数 | 原始值 | 保守值 | 推荐值 | 激进值 | 说明 |
|------|--------|--------|--------|--------|------|
| **特征跟踪** | | | | | |
| max_cnt | 100 | 60 | 80 | 100 | 特征点数量 |
| min_dist | 10 | 20 | 15 | 12 | 特征点间距 |
| F_threshold | 1.0 | 1.8 | 1.5 | 1.2 | 图像RANSAC阈值 |
| F_threshold_1 | 0.7 | 1.2 | 1.0 | 0.8 | Event RANSAC阈值 |
| equalize | 1 | 1 | 1 | 1 | CLAHE增强 (必须开启) |
| CLAHE clipLimit | 3.0 | 2.0 | 2.5 | 3.5 | 对比度增强强度 (代码修改) |
| 角点质量阈值 | 0.01 | 0.03 | 0.02 | 0.015 | goodFeaturesToTrack (代码修改) |
| 光流窗口 | 21 | 31 | 25 | 21 | calcOpticalFlowPyrLK (代码修改) |
| **优化鲁棒性（新增）** | | | | | |
| visual_loss_threshold | 1.0 | 8.0 | 5.0 | 3.0 | 视觉约束误差阈值（像素） |
| use_hard_threshold | - | 1 | 1 | 0 | 核函数类型（0=Cauchy, 1=硬阈值, 2=饱和） |

---

## 调试技巧

### 1. 可视化特征跟踪
确保 `show_track: 1`，然后查看话题：
```bash
rosrun rqt_image_view rqt_image_view /feature_tracker/feature_img
```

**观察指标**:
- 绿色轨迹：跟踪成功的特征点
- 轨迹长度：长轨迹表示稳定特征
- 特征分布：应均匀分布，避免聚集

### 2. 检查特征点数量
```bash
rostopic echo /feature_tracker/feature
# 查看每帧的特征点数量和跟踪ID
```

**健康状态**:
- 特征点数量稳定在50-80之间
- ID连续性好，说明跟踪稳定
- 如果频繁出现新ID，说明跟踪丢失严重

### 3. 录制对比数据
```bash
# 录制调优前后的特征跟踪结果
rosbag record /feature_tracker/feature_img /feature_tracker/feature -O tuning_test.bag
```

---

## 常见问题与解决方案

### Q1: 特征点大量聚集在悬浮物上
**症状**: 特征点集中在噪点区域，真实场景特征很少  
**解决**:
1. 增加 `min_dist` 到 20-25
2. 提高角点质量阈值到 0.02-0.03
3. 使用预处理节点进行去噪

### Q2: 特征点跟踪频繁丢失
**症状**: 特征点ID频繁更新，轨迹很短  
**解决**:
1. 降低 `F_threshold` (但不低于1.0)
2. 增加 `max_cnt` 到 100
3. 检查图像质量，是否需要更强的CLAHE

### Q3: SLAM漂移严重
**症状**: 轨迹与真值偏差大  
**解决**:
1. 提高 `F_threshold` 到 1.8-2.0，严格剔除误匹配
2. 检查IMU参数是否准确
3. 考虑启用DVL辅助 (`dvl_add: 1`)

### Q4: 处理速度太慢
**症状**: 特征跟踪频率低于5Hz  
**解决**:
1. 降低 `max_cnt` 到 60
2. 降低 `freq` (但不低于5)
3. 代码级优化：减小光流窗口和金字塔层数

---

## 总结

**立即生效的配置** (已应用到canghewan.yaml):
```yaml
max_cnt: 80
min_dist: 15
F_threshold: 1.5
F_threshold_1: 1.0
equalize: 1
```

**如果需要进一步优化**，按以下顺序尝试：
1. YAML参数微调 (5分钟)
2. 启用图像预处理节点 (10分钟)
3. 代码级参数修改 (30分钟 + 编译)

**验证方法**:
- 运行rosbag，观察 `/feature_tracker/feature_img`
- 检查特征点分布是否均匀
- 检查跟踪轨迹长度是否增加
- 最终检查SLAM轨迹精度

---

## 参考资料

- VINS-Mono论文: https://arxiv.org/abs/1708.03852
- OpenCV光流文档: https://docs.opencv.org/master/dc/d6b/group__video__track.html
- CLAHE参数选择: https://docs.opencv.org/master/d5/daf/tutorial_py_histogram_equalization.html

最后更新: 2025-11-12
