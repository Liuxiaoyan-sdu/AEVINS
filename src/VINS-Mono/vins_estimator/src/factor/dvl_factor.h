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
          Qi_(Qi), omega_(omega), sqrt_info_(information.llt().matrixL().transpose()) {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        // 参数：para_SpeedBias，9维，前3维是世界坐标系下的速度，后6维是ba和bg
        Eigen::Map<const Eigen::Vector3d> v_world(parameters[0]);        // 世界坐标系速度
        Eigen::Vector3d bg(parameters[0][6], parameters[0][7], parameters[0][8]); // 陀螺仪偏置

        // 将世界坐标系下的速度转换到IMU坐标系
        Eigen::Vector3d v_imu_opt = Qi_.inverse() * v_world;

        // 将DVL测量的速度转换到IMU坐标系
        Eigen::Vector3d v_imu_measured = R_dvl_to_imu_ * v_dvl_;

        // 计算旋转带来的速度影响
        Eigen::Vector3d omega_corrected = omega_ - bg; // 修正后的真实角速度
        Eigen::Vector3d v_rotation = omega_corrected.cross(t_dvl_to_imu_); // 旋转引起的附加速度
        Eigen::Vector3d v_imu_corrected = v_imu_measured;// - v_rotation; // 修正后的DVL速度
        // 计算残差
        
        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual = v_imu_opt - v_imu_corrected;
        residual = sqrt_info_ * residual; // 加权残差
        // cout<<"residual: "<<residual.transpose()<<endl;
        // cout<<"v_imu_opt: "<<v_imu_opt.transpose()<<endl;
        // cout<<"v_imu_corrected: "<<v_imu_corrected.transpose()<<endl;
       

        // 计算雅可比矩阵（如果需要）
        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> jacobian(jacobians[0]);
            jacobian.setZero();
            // 残差对 v_world 的偏导数
            jacobian.block<3, 3>(0, 0) = sqrt_info_ * Qi_.inverse().toRotationMatrix();
            // 对 bg 的偏导数（通过 v_rotation 间接影响）
            Eigen::Matrix3d skew_t = skewSymmetric(t_dvl_to_imu_);
            jacobian.block<3, 3>(0, 6) = sqrt_info_ * skew_t;
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
};
