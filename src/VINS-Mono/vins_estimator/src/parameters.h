#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR
extern int DVL_ADD;
extern int DVL_POSE_ADD;
extern Eigen::Matrix3d R_DVL_TO_IMU;
extern Eigen::Vector3d T_DVL_TO_IMU;
extern Eigen::Matrix3d INFORMATION_MATRIX;

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;

// 视觉鲁棒核函数参数（用于水下虚无区域的外点剔除）
extern double VISUAL_LOSS_THRESHOLD;  // 视觉重投影误差阈值（像素）
extern int USE_HARD_THRESHOLD;         // 是否使用硬阈值（超过阈值则权重为0）
extern std::string IMU_TOPIC;
extern std::string DVL_TOPIC;
extern std::string DVL_POSITION_TOPIC;

extern double TD;
extern double TD_DVL;  // DVL时间偏移
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;
extern Eigen::Matrix<double, 6, 1> INFO_VECTOR_DVL_POSE;


void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
