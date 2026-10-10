#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Matrix3d R_DVL_TO_IMU;
Eigen::Vector3d T_DVL_TO_IMU;
Eigen::Matrix3d INFORMATION_MATRIX;
Eigen::Matrix<double, 6, 1> INFO_VECTOR_DVL_POSE;

Eigen::Vector3d G{0.0, 0.0, 9.8};


int DVL_ADD;
int DVL_POSE_ADD ;
int USE_DVL_CAUCHY_LOSS = 0;
double DVL_CAUCHY_SCALE = 0.5;
int DVL_CAUCHY_DELAY_FRAMES = 20;  // 初始化后延迟20帧才启用柯西损失

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
std::string DVL_POSITION_TOPIC;
std::string DVL_TOPIC;  
double ROW, COL;
double TD, TD_DVL, TR;

// 视觉鲁棒核函数参数
double VISUAL_LOSS_THRESHOLD = 3.0;  // 默认3.0像素
int USE_HARD_THRESHOLD = 0;           // 默认使用软核函数（Cauchy）

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["dvl_topic"] >> DVL_TOPIC;
    fsSettings["dvl_position_topic"] >> DVL_POSITION_TOPIC;

    DVL_ADD = fsSettings["dvl_add"];
    DVL_POSE_ADD = fsSettings["dvl_pose_add"];
    ROS_INFO("DVL_ADD: %d", DVL_ADD);
    ROS_INFO("DVL_POSE_ADD: %d", DVL_POSE_ADD);
    // 读取DVL损失函数参数
    if (fsSettings["use_dvl_cauchy_loss"].isInt())
        USE_DVL_CAUCHY_LOSS = fsSettings["use_dvl_cauchy_loss"];
    if (fsSettings["dvl_cauchy_scale"].isReal())
        DVL_CAUCHY_SCALE = fsSettings["dvl_cauchy_scale"];
    if (fsSettings["dvl_cauchy_delay_frames"].isInt())
        DVL_CAUCHY_DELAY_FRAMES = fsSettings["dvl_cauchy_delay_frames"];
    ROS_INFO("USE_DVL_CAUCHY_LOSS: %d, DVL_CAUCHY_SCALE: %.4f, DELAY_FRAMES: %d", 
             USE_DVL_CAUCHY_LOSS, DVL_CAUCHY_SCALE, DVL_CAUCHY_DELAY_FRAMES);
    if (DVL_ADD == 1||DVL_POSE_ADD == 1)
    {
        fsSettings["dvl_topic"] >> DVL_TOPIC;
        if (DVL_TOPIC.empty())
        {
            ROS_ERROR("Failed to load dvl_topic");
            n.shutdown();
            return;
        }

        cv::Mat cv_R, cv_T, cv_I;
        fsSettings["rdvltoimu"] >> cv_R;
        if (cv_R.empty() || cv_R.rows != 3 || cv_R.cols != 3)
        {
            ROS_ERROR("Failed to load Rdvltoimu or invalid dimensions");
            n.shutdown();
            return;
        }
        fsSettings["tdvltoimu"] >> cv_T;
        if (cv_T.empty() || cv_T.rows != 3 || cv_T.cols != 1)
        {
            ROS_ERROR("Failed to load Tdvltoimu or invalid dimensions");
            n.shutdown();
            return;
        }
        fsSettings["dvl_information_matrix"] >> cv_I;
        if (cv_I.empty() || cv_I.rows != 3 || cv_I.cols != 3)
        {
            ROS_ERROR("Failed to load dvl_information_matrix or invalid dimensions");
            n.shutdown();
            return;
        }

        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        Eigen::Matrix3d eigen_information_matrix;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        cv::cv2eigen(cv_I, eigen_information_matrix);
        R_DVL_TO_IMU = eigen_R;
        T_DVL_TO_IMU = eigen_T;
        INFORMATION_MATRIX = eigen_information_matrix;
        ROS_INFO_STREAM("R_DVL_TO_IMU:\n" << R_DVL_TO_IMU);
        ROS_INFO("DVL_TOPIC: %s", DVL_TOPIC.c_str());
        ROS_INFO("add dvl factor into the optimization problem");
    }
    
    if (DVL_POSE_ADD == 1){
        cv::Mat cv_PI;
        fsSettings["dvlpose_information_matrix"] >> cv_PI;
        if (cv_PI.empty()) {
        ROS_ERROR("Failed to load dvlpose_information_matrix, expected a 6x1 vector");
        n.shutdown();
        return;
    }
        Eigen::Matrix<double, 6, 1> eigen_PI;
        cv::cv2eigen(cv_PI, eigen_PI);
        INFO_VECTOR_DVL_POSE = eigen_PI;
    }


    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    // 读取视觉鲁棒核函数参数（水下虚无区域外点剔除）
    if (fsSettings["visual_loss_threshold"].isReal())
    {
        VISUAL_LOSS_THRESHOLD = fsSettings["visual_loss_threshold"];
        ROS_INFO("Visual loss threshold: %f pixels", VISUAL_LOSS_THRESHOLD);
    }
    else
    {
        VISUAL_LOSS_THRESHOLD = 3.0;  // 默认3.0像素
        ROS_WARN("visual_loss_threshold not found, using default: 3.0 pixels");
    }

    if (fsSettings["use_hard_threshold"].isInt())
    {
        USE_HARD_THRESHOLD = (int)fsSettings["use_hard_threshold"];
        ROS_INFO("Use hard threshold: %d", USE_HARD_THRESHOLD);
    }
    else
    {
        USE_HARD_THRESHOLD = 0;  // 默认使用软核函数
        ROS_WARN("use_hard_threshold not found, using default: 0 (soft loss)");
    }

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    // 读取DVL时间偏移参数
    if (fsSettings["td_dvl"].isReal())
    {
        TD_DVL = fsSettings["td_dvl"];
        ROS_INFO_STREAM("DVL time offset: " << TD_DVL);
    }
    else
    {
        TD_DVL = 0.0;
        ROS_WARN("td_dvl not found in config, using default: 0.0");
    }

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
}
