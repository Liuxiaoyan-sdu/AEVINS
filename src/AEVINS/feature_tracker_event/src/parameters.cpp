#include "parameters.h"

std::string IMAGE0_TOPIC;
std::string IMAGE1_TOPIC;

std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
double F_THRESHOLD_1;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

// Event feature control flags
int USE_EVENT_COMPENSATION = 1;  // Default: no compensation
int USE_DYNAMIC_WINDOW = 1;      // Default: fixed window
int USE_IMAGE_FEATURES = 1;      // Default: enable image features
int USE_EVENT_FEATURES = 1;      // Default: enable event features
double IMAGE_ENHANCE_GAMMA = 1.0; // Default: no gamma correction

// Velocity-adaptive event collection parameters
double VELOCITY_ADAPTIVE_BETA  = 2.0;  // Default beta
double VELOCITY_ADAPTIVE_REF   = 0.5;  // Default reference speed (m/s)

// Feature statistics publishing
int PUBLISH_FEATURE_STATS = 0;
int FEATURE_TRACK_MIN_CNT = 5;

// Per-frame timing statistics publishing
int PUBLISH_TIMING_STATS = 0;
// Backend optimization timing publish control
int PUBLISH_BACKEND_TIMING = 0;

// Dynamic window parameters
int MIN_EVENT_COUNT = 2000;
int MAX_EVENT_COUNT = 20000;
int INITIAL_EVENT_COUNT = 5000;
double ALPHA = 0.1;
double MIN_PIXEL_RATIO = 0.2;
int EVENT_HISTORY_SIZE = 1000000;
unsigned long long INTERVAL_TIME_NS = 80000000;

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
    std::string AEVINS_FOLDER_PATH = readParam<std::string>(n, "aevins_folder");

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;

    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    F_THRESHOLD_1 = fsSettings["F_threshold_1"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = AEVINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    
    // Read event feature control flags
    USE_EVENT_COMPENSATION = fsSettings["use_event_compensation"];
    USE_DYNAMIC_WINDOW = fsSettings["use_dynamic_window"];
    USE_IMAGE_FEATURES = fsSettings["use_image_features"];
    USE_EVENT_FEATURES = fsSettings["use_event_features"];
    // Gamma correction for image brightness
    IMAGE_ENHANCE_GAMMA = fsSettings["image_enhance_gamma"];
    if (IMAGE_ENHANCE_GAMMA < 0.1) IMAGE_ENHANCE_GAMMA = 1.0; // Safety check

    // Velocity-adaptive event collection parameters
    {
        cv::FileNode node = fsSettings["velocity_adaptive_beta"];
        VELOCITY_ADAPTIVE_BETA = node.empty() ? 2.0 : (double)node;
        node = fsSettings["velocity_adaptive_ref"];
        VELOCITY_ADAPTIVE_REF  = node.empty() ? 0.5 : (double)node;
    }
    ROS_INFO("Velocity-adaptive: beta=%.3f, v_ref=%.3f m/s",
             VELOCITY_ADAPTIVE_BETA, VELOCITY_ADAPTIVE_REF);

    // Feature statistics publishing
    {
        cv::FileNode node = fsSettings["publish_feature_stats"];
        PUBLISH_FEATURE_STATS = node.empty() ? 0 : (int)node;
        node = fsSettings["feature_track_min_cnt"];
        FEATURE_TRACK_MIN_CNT = node.empty() ? 5 : (int)node;
        node = fsSettings["publish_timing_stats"];
        PUBLISH_TIMING_STATS = node.empty() ? 0 : (int)node;
        node = fsSettings["publish_backend_timing"];
        PUBLISH_BACKEND_TIMING = node.empty() ? 0 : (int)node;
    }
    ROS_INFO("Publish feature stats: %d (min_track_cnt=%d), Publish timing stats: %d",
             PUBLISH_FEATURE_STATS, FEATURE_TRACK_MIN_CNT, PUBLISH_TIMING_STATS);
    ROS_INFO("Publish backend timing: %d", PUBLISH_BACKEND_TIMING);
    
    // Read dynamic window parameters
    MIN_EVENT_COUNT = fsSettings["min_event_count"];
    MAX_EVENT_COUNT = fsSettings["max_event_count"];
    INITIAL_EVENT_COUNT = fsSettings["initial_event_count"];
    ALPHA = fsSettings["alpha"];
    MIN_PIXEL_RATIO = fsSettings["min_pixel_ratio"];
    EVENT_HISTORY_SIZE = fsSettings["event_history_size"];
    int interval_time_ms;
    fsSettings["interval_time_ns"] >> interval_time_ms;
    INTERVAL_TIME_NS = static_cast<unsigned long long>(interval_time_ms);
    
    ROS_INFO("Event compensation: %d, Dynamic window: %d", USE_EVENT_COMPENSATION, USE_DYNAMIC_WINDOW);
    ROS_INFO("Use image features: %d, Use event features: %d", USE_IMAGE_FEATURES, USE_EVENT_FEATURES);
    ROS_INFO("Image enhance gamma: %.2f", IMAGE_ENHANCE_GAMMA);
    ROS_INFO("Event params - Min: %d, Max: %d, Initial: %d, History: %d", 
             MIN_EVENT_COUNT, MAX_EVENT_COUNT, INITIAL_EVENT_COUNT, EVENT_HISTORY_SIZE);
    
    //CAM_NAMES.push_back(config_file);
    // camera config
    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    std::string cam1Calib;
    fsSettings["cam1_calib"] >> cam1Calib;
    std::string cam1Path = configPath + "/" + cam1Calib; 
    CAM_NAMES.push_back(cam1Path);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = true;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}