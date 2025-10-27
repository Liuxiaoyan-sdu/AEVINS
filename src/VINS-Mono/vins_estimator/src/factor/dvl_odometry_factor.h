#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include <ceres/ceres.h>

class DVLOdometryFactor : public ceres::SizedCostFunction<6, 7, 7>
{
public:
    DVLOdometryFactor() = delete;
    DVLOdometryFactor(const Eigen::Vector3d& _delta_p_imu,      // IMU坐标系下的相对位移
                     const Eigen::Quaterniond& _delta_q_imu,    // IMU坐标系下的相对姿态
                     const Eigen::Matrix<double, 6, 6>& information) // 协方差矩阵
        : delta_p_imu_(_delta_p_imu), 
          delta_q_imu_(_delta_q_imu), 
          sqrt_info_(information.llt().matrixL().transpose()) {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
    {
        // 第i帧的参数
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        // 第j帧的参数
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        // 计算估计的相对运动（在i帧IMU坐标系下）
        Eigen::Quaterniond delta_q_est = Qi.inverse() * Qj;
        Eigen::Vector3d delta_p_est = Qi.inverse() * (Pj - Pi);

        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);

        // 位移残差（都在i帧IMU坐标系下）
        residual.segment<3>(0) = delta_p_est - delta_p_imu_;

        // 姿态残差（四元数差的虚部）
        Eigen::Quaterniond q_error = delta_q_imu_.inverse() * delta_q_est;
        residual.segment<3>(3) = 2.0 * q_error.vec();

        // 加权残差
        residual = sqrt_info_ * residual;

        // 计算雅可比矩阵
        if (jacobians)
        {
            if (jacobians[0]) // 对第i帧 (Pi, Qi)
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                // ∂r_position/∂Pi
                jacobian_pose_i.block<3, 3>(0, 0) = -Qi.inverse().toRotationMatrix();

                // ∂r_position/∂Qi
                Eigen::Vector3d p_ij = Pj - Pi;
                jacobian_pose_i.block<3, 3>(0, 3) = Utility::skewSymmetric(Qi.inverse() * p_ij);

                // ∂r_rotation/∂Qi
                jacobian_pose_i.block<3, 3>(3, 3) = -Utility::Qleft(Qj.inverse() * Qi).bottomRightCorner<3, 3>();

                jacobian_pose_i = sqrt_info_ * jacobian_pose_i;
            }

            if (jacobians[1]) // 对第j帧 (Pj, Qj)
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
                jacobian_pose_j.setZero();

                // ∂r_position/∂Pj
                jacobian_pose_j.block<3, 3>(0, 0) = Qi.inverse().toRotationMatrix();

                // ∂r_rotation/∂Qj
                jacobian_pose_j.block<3, 3>(3, 3) = Utility::Qleft(delta_q_imu_.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();

                jacobian_pose_j = sqrt_info_ * jacobian_pose_j;
            }
        }

        return true;
    }

private:
    Eigen::Vector3d delta_p_imu_;      // IMU坐标系下的相对位移
    Eigen::Quaterniond delta_q_imu_;   // IMU坐标系下的相对姿态
    Eigen::Matrix<double, 6, 6> sqrt_info_; // 信息矩阵的平方根
};