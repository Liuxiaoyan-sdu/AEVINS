#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 2;


extern std::string IMAGE0_TOPIC;
extern std::string IMAGE1_TOPIC;

extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern double F_THRESHOLD_1;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

// Event feature control flags
extern int USE_EVENT_COMPENSATION;  // 0: no compensation, 1: use motion compensation
extern int USE_DYNAMIC_WINDOW;      // 0: fixed window, 1: adaptive dynamic window
extern int USE_IMAGE_FEATURES;      // 0: disable, 1: enable image features
extern int USE_EVENT_FEATURES;      // 0: disable, 1: enable event features
extern double IMAGE_ENHANCE_GAMMA;  // Gamma correction value

// Velocity-adaptive event collection parameters
// Adjusts previous frame event count: V_{j-1,new} = V_{j-1} * (1 + beta*v_ref) / (1 + beta*v_cur)
// Used in dynamic window calculation to adapt event collection to current speed
extern double VELOCITY_ADAPTIVE_BETA;   // beta: sensitivity to speed (m/s)^-1
extern double VELOCITY_ADAPTIVE_REF;    // v_ref: reference speed (m/s)

// Feature statistics publishing
// publish_feature_stats: 0=off, 1=on
// Publishes a stamped Float64MultiArray(4):
//   [0] total image features  [1] tracked image features (track_cnt >= feature_track_min_cnt)
//   [2] total event features  [3] tracked event features
extern int PUBLISH_FEATURE_STATS;
extern int FEATURE_TRACK_MIN_CNT;    // minimum continuous tracking frames to count as "tracked"

// Per-frame timing statistics publishing
// publish_timing_stats: 0=off, 1=on
// Publishes a stamped Float64MultiArray:
//   data[0] = feature extraction time (ms, image+event combined)
//   data[1] = feature matching time (ms)
//   data[2] = motion compensation time (ms, only recompute())
//   data[3] = system slice time (ms) (time to build event slice)
extern int PUBLISH_TIMING_STATS;

// Backend optimization timing: subscribe/republish backend optimization duration
// publish_backend_timing: 0=off, 1=on
extern int PUBLISH_BACKEND_TIMING;

// Dynamic window parameters
extern int MIN_EVENT_COUNT;
extern int MAX_EVENT_COUNT;
extern int INITIAL_EVENT_COUNT;
extern double ALPHA;
extern double MIN_PIXEL_RATIO;
extern int EVENT_HISTORY_SIZE;
extern unsigned long long INTERVAL_TIME_NS;

void readParameters(ros::NodeHandle &n);

#endif