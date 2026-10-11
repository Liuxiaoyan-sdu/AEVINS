#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>
#include "../utility/utility.h"
#include "../parameters.h"
using namespace Eigen;

class DVLSpeedFactor : public ceres::SizedCostFunction<3, 9> {
public:
    DVLSpeedFactor(const Eigen::Vector3d& v_dvl,           // DVL测量的速度（DVL坐标系）
                   const Eigen::Matrix3d& R_dvl_to_imu,   // DVL到IMU的旋转矩阵
                   const Eigen::Vector3d& t_dvl_to_imu,   // DVL到IMU的平移向量
                   const Eigen::Quaterniond& Qi,          // IMU的姿态（世界到IMU）
                   const Eigen::Vector3d& omega,          // IMU的角速度（IMU坐标系）
                   const Eigen::Matrix3d& information)    // 信息矩阵
        : v_dvl_(v_dvl), R_dvl_to_imu_(R_dvl_to_imu), t_dvl_to_imu_(t_dvl_to_imu),
          Qi_(Qi), omega_(omega), sqrt_info_(information.llt().matrixL().transpose()) {
        // 计算sqrt_info的有效尺度（取对角线最大值）
        sqrt_info_scale_ = sqrt_info_.diagonal().maxCoeff();
    }

    // 获取调整后的柯西损失参数
    // 由于残差被sqrt_info_放大，柯西参数也应该相应放大
    double GetAdjustedCauchyScale(double original_scale) const {
        return original_scale * sqrt_info_scale_;
    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        // 参数：para_SpeedBias[9] = [v_world(3), ba(3), bg(3)]
        // 索引: [0-2]: 世界坐标系速度, [3-5]: 加速度计偏置, [6-8]: 陀螺仪偏置
        Eigen::Map<const Eigen::Vector3d> v_world(parameters[0]);  // 世界坐标系速度 [0-2]
        // 注意：不考虑旋转修正时，不需要读取 bg

        // 将世界坐标系下的速度转换到IMU坐标系
        Eigen::Vector3d v_imu_opt = Qi_.inverse() * v_world;

        // 将DVL测量的速度转换到IMU坐标系
        // 注意：不考虑旋转修正，即假设 DVL 和 IMU 在同一点或旋转影响可忽略
        Eigen::Vector3d v_imu_measured = R_dvl_to_imu_ * v_dvl_;

        // 计算残差：估计速度 - 测量速度
        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual = v_imu_opt - v_imu_measured;
        residual = sqrt_info_ * residual; // 加权残差

        // 计算雅可比矩阵
        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> jacobian(jacobians[0]);
            jacobian.setZero();
            
            // 残差对 v_world 的偏导数: ∂r/∂v_world = ∂(Qi^{-1} * v_world)/∂v_world = Qi^{-1}
            jacobian.block<3, 3>(0, 0) = sqrt_info_ * Qi_.inverse().toRotationMatrix();
            
            // 对 ba 的偏导数为 0（速度残差不依赖加速度计偏置）
            // jacobian.block<3, 3>(0, 3) = 0;  // 已经通过 setZero() 设置
            
            // 对 bg 的偏导数为 0（不考虑旋转修正时，残差不依赖陀螺仪偏置）
            // jacobian.block<3, 3>(0, 6) = 0;  // 已经通过 setZero() 设置
        }

        return true;
    }

private:
    // 工具函数：计算反对称矩阵
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) const {
        Eigen::Matrix3d skew;
        skew <<  0,   -v.z(),  v.y(),
                v.z(),  0,   -v.x(),
               -v.y(), v.x(),  0;
        return skew;
    }

    Eigen::Vector3d v_dvl_;          // DVL测量的速度
    Eigen::Matrix3d R_dvl_to_imu_;   // DVL到IMU的旋转
    Eigen::Vector3d t_dvl_to_imu_;   // DVL到IMU的平移
    Eigen::Quaterniond Qi_;          // IMU姿态
    Eigen::Vector3d omega_;          // IMU角速度
    Eigen::Matrix3d sqrt_info_;      // 信息矩阵的Cholesky分解
    double sqrt_info_scale_;         // sqrt_info对角线的最大值，用于调整柯西损失参数
};
