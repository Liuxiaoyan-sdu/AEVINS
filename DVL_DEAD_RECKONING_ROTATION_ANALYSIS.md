# DVL航迹推算（Dead Reckoning）旋转计算详解

## 概述

在VINS-Mono系统中，DVL航迹推算通过 `DVLOdometryFactor` 实现，主要约束相邻帧之间的相对位姿。这个因子对位置和旋转都进行约束。

---

## 核心代码位置

- **因子定义**: `src/VINS-Mono/vins_estimator/src/factor/dvl_odometry_factor.h`
- **使用位置**: `src/VINS-Mono/vins_estimator/src/estimator.cpp` (第880-910行)

---

## 1. DVL航迹推算的数据来源

DVL提供的里程计数据包含：
- **位置**: `(px, py, pz)` - DVL坐标系下的位置
- **姿态**: `(qw, qx, qy, qz)` - DVL坐标系下的姿态四元数

这些数据来自外部DVL设备的航迹推算（通常由DVL设备内部完成）。

---

## 2. 旋转计算的完整流程

### 2.1 获取DVL航迹推算数据

```cpp
// 从dvl_pose_buf中获取相邻两帧的位姿
Eigen::VectorXd pose_i = dvl_pose_buf[i];  // 第i帧
Eigen::VectorXd pose_j = dvl_pose_buf[j];  // 第j帧 (j = i + 1)

// 提取位置
Eigen::Vector3d p_i(pose_i(0), pose_i(1), pose_i(2));
Eigen::Vector3d p_j(pose_j(0), pose_j(1), pose_j(2));

// 提取姿态（四元数: qw, qx, qy, qz）
Eigen::Quaterniond q_i(pose_i(6), pose_i(3), pose_i(4), pose_i(5));
Eigen::Quaterniond q_j(pose_j(6), pose_j(3), pose_j(4), pose_j(5));
```

**注意**: 这里的位姿是DVL设备在**世界坐标系**下的位姿（DVL自己的航迹推算结果）。

---

### 2.2 计算DVL坐标系下的相对运动

#### 相对位移（世界坐标系）
```cpp
Eigen::Vector3d delta_p_dvl_world = p_j - p_i;
```

这是从第i帧到第j帧，在世界坐标系下的位移向量。

#### 相对旋转（世界坐标系）
```cpp
Eigen::Quaterniond delta_q_dvl = q_i.inverse() * q_j;
```

这是从第i帧到第j帧的**相对旋转**，表示：
- `q_i.inverse()`: 从世界坐标系转到第i帧的DVL坐标系
- 乘以 `q_j`: 再转到第j帧的世界坐标系
- **结果**: 从第i帧DVL坐标系到第j帧DVL坐标系的旋转

**数学表达式**:
```
δq_dvl = q_i^(-1) ⊗ q_j
```

其中 `⊗` 表示四元数乘法。

---

### 2.3 坐标系转换：DVL → IMU

由于VINS-Mono优化的是IMU坐标系下的状态，需要将DVL的相对运动转换到IMU坐标系。

#### 相对位移转换
```cpp
Eigen::Matrix3d R_Dk_D = q_i.toRotationMatrix();  // 第i帧DVL姿态的旋转矩阵
Eigen::Vector3d delta_p_imu = (R_Dk_D * R_DVL_TO_IMU).transpose() * delta_p_dvl_world;
```

**转换步骤**:
1. `R_Dk_D`: 第i帧DVL在世界坐标系下的旋转矩阵（世界→DVL_i）
2. `R_DVL_TO_IMU`: DVL坐标系到IMU坐标系的外参旋转矩阵
3. `R_Dk_D * R_DVL_TO_IMU`: 世界坐标系到第i帧IMU坐标系的旋转（世界→IMU_i）
4. `(...)^T`: 转置，得到 IMU_i→世界 的旋转
5. 最后乘以世界坐标系下的位移，得到IMU坐标系下的相对位移

**数学表达式**:
```
δp_imu = (R_world→IMU_i)^T * δp_world
       = R_IMU_i→world * δp_world
```

#### 相对旋转转换（重点！）
```cpp
Eigen::Matrix3d delta_q_imu_mat = R_DVL_TO_IMU.transpose() * delta_q_dvl.toRotationMatrix() * R_DVL_TO_IMU;
Eigen::Quaterniond delta_q_imu(delta_q_imu_mat);
```

**转换逻辑**（这是最关键的部分）:

假设有一个向量 `v`，在不同坐标系下有不同的表示：
- `v_dvl_i`: 在第i帧DVL坐标系下的表示
- `v_dvl_j`: 在第j帧DVL坐标系下的表示
- `v_imu_i`: 在第i帧IMU坐标系下的表示
- `v_imu_j`: 在第j帧IMU坐标系下的表示

**在DVL坐标系**:
```
v_dvl_j = R_dvl * v_dvl_i
```
其中 `R_dvl` 是 `delta_q_dvl` 的旋转矩阵。

**转换到IMU坐标系**:
```
v_imu_i = R_DVL_TO_IMU * v_dvl_i
v_imu_j = R_DVL_TO_IMU * v_dvl_j
        = R_DVL_TO_IMU * R_dvl * v_dvl_i
        = R_DVL_TO_IMU * R_dvl * R_DVL_TO_IMU^T * v_imu_i
```

因此，IMU坐标系下的相对旋转为:
```
R_imu = R_DVL_TO_IMU * R_dvl * R_DVL_TO_IMU^T
```

这就是代码中的:
```cpp
R_DVL_TO_IMU.transpose() * delta_q_dvl.toRotationMatrix() * R_DVL_TO_IMU
```

**直观理解**:
1. `R_DVL_TO_IMU`: 把IMU坐标系下的向量转到DVL坐标系
2. `delta_q_dvl.toRotationMatrix()`: 在DVL坐标系下应用旋转
3. `R_DVL_TO_IMU.transpose()`: 再转回IMU坐标系

这种转换称为**相似变换**（Similarity Transformation）或**坐标系变换**。

---

## 3. 残差计算（DVLOdometryFactor）

在 `DVLOdometryFactor::Evaluate()` 中计算残差：

### 3.1 计算优化估计的相对运动

```cpp
// 优化变量中的第i帧和第j帧位姿
Eigen::Vector3d Pi, Pj;           // 位置
Eigen::Quaterniond Qi, Qj;        // 姿态

// 计算估计的相对运动（在第i帧IMU坐标系下）
Eigen::Quaterniond delta_q_est = Qi.inverse() * Qj;
Eigen::Vector3d delta_p_est = Qi.inverse() * (Pj - Pi);
```

### 3.2 计算残差

#### 位移残差
```cpp
residual.segment<3>(0) = delta_p_est - delta_p_imu_;
```

**含义**: 优化估计的相对位移 vs DVL测量的相对位移（都在第i帧IMU坐标系下）

#### 旋转残差
```cpp
Eigen::Quaterniond q_error = delta_q_imu_.inverse() * delta_q_est;
residual.segment<3>(3) = 2.0 * q_error.vec();
```

**含义**: 
- `q_error`: 旋转误差四元数
- `q_error.vec()`: 四元数的虚部（xyz分量），表示旋转误差的轴角表示
- `2.0 * q_error.vec()`: 对应小角度近似下的旋转误差向量

**数学原理**:
对于小旋转，四元数可以近似为:
```
q ≈ [1, 0.5*θ]^T
```
其中 `θ` 是旋转向量（轴角表示）。因此:
```
旋转误差 ≈ 2 * vec(q_error)
```

---

## 4. 关键公式总结

### DVL坐标系下的相对旋转
```
δq_dvl = q_i^(-1) ⊗ q_j
```

### 转换到IMU坐标系
```
R_imu = R_DVL_TO_IMU * R_dvl * R_DVL_TO_IMU^T
```

或用四元数表示:
```
δq_imu = q_DVL_TO_IMU ⊗ δq_dvl ⊗ q_DVL_TO_IMU^(-1)
```

### 残差计算
**位移残差**:
```
r_p = Q_i^(-1) * (P_j - P_i) - δp_imu
```

**旋转残差**:
```
r_q = 2 * vec(δq_imu^(-1) ⊗ Q_i^(-1) ⊗ Q_j)
```

---

## 5. 坐标系说明

### 涉及的坐标系
1. **世界坐标系 (World)**: 优化的参考坐标系
2. **DVL坐标系 (DVL)**: DVL设备的本体坐标系
3. **IMU坐标系 (IMU)**: IMU设备的本体坐标系
4. **第i帧坐标系**: 第i时刻的本体坐标系

### 外参
- `R_DVL_TO_IMU`: DVL坐标系到IMU坐标系的旋转矩阵（外参，从配置文件读取）
- `T_DVL_TO_IMU`: DVL到IMU的平移向量（本代码中未使用）

---

## 6. 旋转转换的几何意义

### 为什么需要相似变换？

当我们有一个旋转 `R_dvl`，它表示在DVL坐标系下的旋转。如果我们想知道同样的物理旋转在IMU坐标系下是什么样的，我们需要进行坐标系转换。

**类比**:
假设你在一个房间里旋转了90度。如果以房间为参考系，你旋转了90度。但如果以你自己为参考系，房间旋转了-90度。相似变换就是在不同参考系下描述同一个物理旋转。

### 相似变换公式的推导

设向量 `v` 经过旋转后变为 `v'`:
- 在DVL坐标系: `v'_dvl = R_dvl * v_dvl`
- 在IMU坐标系: `v'_imu = R_imu * v_imu`

坐标系转换关系:
```
v_imu = R_DVL_TO_IMU * v_dvl
v'_imu = R_DVL_TO_IMU * v'_dvl
```

代入第一个方程:
```
v'_imu = R_DVL_TO_IMU * v'_dvl
       = R_DVL_TO_IMU * R_dvl * v_dvl
       = R_DVL_TO_IMU * R_dvl * R_DVL_TO_IMU^T * v_imu
```

因此:
```
R_imu = R_DVL_TO_IMU * R_dvl * R_DVL_TO_IMU^T
```

---

## 7. 代码调试建议

如果想验证旋转转换是否正确，可以在代码中添加输出：

```cpp
// 在 estimator.cpp 的 DVL_POSE_ADD 部分添加
ROS_INFO_STREAM("Frame " << i << " to " << j << " DVL Odometry:");
ROS_INFO_STREAM("DVL相对旋转 (world): " << delta_q_dvl.w() << ", " 
                << delta_q_dvl.x() << ", " << delta_q_dvl.y() << ", " << delta_q_dvl.z());
ROS_INFO_STREAM("IMU相对旋转 (world): " << delta_q_imu.w() << ", " 
                << delta_q_imu.x() << ", " << delta_q_imu.y() << ", " << delta_q_imu.z());
ROS_INFO_STREAM("旋转转换矩阵:\n" << delta_q_imu_mat);
```

---

## 8. 常见问题

### Q1: 为什么相对旋转是 `q_i.inverse() * q_j` 而不是 `q_j * q_i.inverse()`?

**答**: 这涉及四元数乘法的方向。
- `q_i`: 世界→第i帧
- `q_j`: 世界→第j帧
- `q_i.inverse() * q_j`: 第i帧→世界→第j帧 = 第i帧→第j帧

### Q2: 为什么位移转换用转置，而旋转转换用相似变换？

**答**: 
- **位移**: 只需要旋转向量本身，所以用 `R^T * v`（将世界坐标系的向量转到IMU坐标系）
- **旋转**: 需要改变旋转矩阵的参考系，所以用相似变换 `R * R_dvl * R^T`

### Q3: `R_DVL_TO_IMU` 的方向是什么？

**答**: 从代码来看，`R_DVL_TO_IMU` 表示 DVL坐标系→IMU坐标系 的旋转矩阵。即：
```
v_imu = R_DVL_TO_IMU * v_dvl
```

---

## 9. 总结

DVL航迹推算的旋转计算核心是**坐标系转换**：

1. **获取DVL航迹推算的相对旋转**（世界坐标系下）
2. **转换到IMU坐标系**（使用相似变换）
3. **在优化中作为约束**（约束相邻帧的相对位姿）

关键公式:
```
R_imu = R_DVL_TO_IMU * R_dvl * R_DVL_TO_IMU^T
```

这确保了DVL的航迹推算信息能正确地融合到IMU坐标系下的VINS优化中。
