//该文件夹包含 ROS 前端代码，它读取摄像机的数据并发布可视化数据，以便在 Rviz 中显示
//动态窗口 - 自适应事件数量
#include <vector>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <iomanip>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <better_flow/common.h>
#include <better_flow/event.h>
#include <better_flow/dvs_flow.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "parameters.h"

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0


#define EVENT_WIDTH 100000
#define TIME_WIDTH 0.5

#define EVENT_WIDTH_PROCESS 40000
#define TIME_WIDTH_PROCESS 0.05

// 自适应事件数量参数
#define MIN_EVENT_COUNT 5000  // 最小事件数量
#define MAX_EVENT_COUNT 20000 // 最大事件数量
#define ALPHA 0.1             // 自适应系数 α
#define PIXEL_PLANE_SIZE 89960 // 像素平面大小 (346*260)
#define MIN_PIXEL_RATIO 0.2   // 最小像素比例 (20%)
#define INITIAL_EVENT_COUNT 10000 // 初始事件数量
#define EVENT_HISTORY_SIZE 1000000 // 事件历史缓冲区大小

using EventBuffer = std::deque<dvs_msgs::Event>;
using EventArray = std::vector<dvs_msgs::Event>;
using EventArrayPtr = std::shared_ptr<EventArray>;

// Node launch parameters (set in main)
float refresh_rate;
std::string input_event_topic;
std::string input_image_topic;
std::string output_image_s0_topic;
bool process_data;
bool init_pub = false;
ull interval_time_ns = 40000000;

// Main class
template <size_t MAX_SZ, sll SPAN> class EventVisualizer {
protected:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;

    // Publishers/Subscribers
    ros::Subscriber event_sub;
    ros::Subscriber image_sub;

    image_transport::Publisher image_pub;
    image_transport::Publisher suppl_image_pub_0;

    // Buffer for incoming events (aka 'slice')是一个循环数组，用于存储最近接收到的事件数据。
    CircularArray<Event, MAX_SZ, SPAN> ev_buffer;

    // Helper variables
    bool first_event_received; //第一个事件是否已经接受

    // on_ev_change 和 on_time_change 用于设置处理事件的触发条件。
    ull on_ev_change, on_time_change;

    // 时间计数器或事件计数器
    sll time_diff, event_diff; // Time passed / new event count 新事件

    // 上一个切片和当前切片的时间
    ull last_slice_time, current_slice_time;

    // Flow estimator / minimizer estimator 是光流估计器，用于处理事件数据以计算光流。
    DVS_flow<EVENT_WIDTH_PROCESS, FROM_SEC(TIME_WIDTH_PROCESS)> *estimator;

    //add mine
    EventBuffer event_buffer_;
    // 改为保存时间戳和图像的组合
    std::deque<std::pair<ros::Time, cv::Mat>> time_buffer_;
    bool first_package = true ;
    bool new_image = false;
    ull cur_fr_stamp, package_start;
    ull last_package_size = 10000;
    ull package_end;
    
    // 自适应事件数量相关变量
    ull current_event_count;  // 当前事件数量 V_j
    ull previous_event_count; // 前一个事件数量 V_{j-1}
    ros::Time first_event_time; // 当前时间窗口中最早事件的时间 t_{first}
    bool is_first_frame;      // 是否为第一帧

    // 新增：保存最近的原始图像及其header
    // cv::Mat last_image_; // Removed
    // std_msgs::Header last_image_header_; // Removed
    // std::mutex image_mutex_; // Removed
    image_transport::Publisher concat_image_pub_;

    bool first_image_flag_ = true;
    double first_image_time_ = 0;
    double last_image_time_ = 0;
    int pub_count_ = 1;
    ros::Publisher& pub_restart_;
    ros::Publisher& pub_merged_;
    ros::Publisher& pub_match_0_;
    ros::Publisher& pub_match_1_;

    FeatureTracker trackerData_img;
    FeatureTracker trackerData_event;

public:
    EventVisualizer (ros::NodeHandle n, ull on_ev_change_, ull on_time_change_,
                 ros::Publisher& pub_merged,
                 ros::Publisher& pub_match_0,
                 ros::Publisher& pub_match_1,
                 ros::Publisher& pub_restart)
    : n_(n), it_(n), first_event_received(false),
      on_ev_change(on_ev_change_), on_time_change(on_time_change_),
      time_diff(0), event_diff(0), last_slice_time(0), current_slice_time(0),
      estimator(NULL), package_end(0), current_event_count(INITIAL_EVENT_COUNT),
      previous_event_count(INITIAL_EVENT_COUNT), is_first_frame(true),
      pub_merged_(pub_merged), pub_match_0_(pub_match_0), pub_match_1_(pub_match_1), pub_restart_(pub_restart) {
        this->event_sub = this->n_.subscribe(input_event_topic, 0, &EventVisualizer::time_rate_slice, this);
        this->image_sub = this->n_.subscribe(input_image_topic, 0, &EventVisualizer::image_slice, this);
        if (process_data) {
            this->estimator = new DVS_flow<EVENT_WIDTH_PROCESS, FROM_SEC(TIME_WIDTH_PROCESS)>
                                          (20000000, LLONG_MAX);//类中汇集了运动补偿流水线的所有底层基元，dvs_flow
            this->estimator->set_stm_disable(false);//启用或禁用 STM 功能。
            this->estimator->set_scale(1);//设置缩放比例为 1。
            this->estimator->set_max_iter(10);//设置缩放比例为 1。
            this->suppl_image_pub_0 = this->it_.advertise(output_image_s0_topic, 1);
            // 新增：拼接图像发布器
            this->concat_image_pub_ = this->it_.advertise("better_flow/concat_image", 1);
        } 
        event_buffer_.clear();
        // 初始化特征提取器参数
        trackerData_img.readIntrinsicParameter(CAM_NAMES, 0);
        trackerData_img.f_threshold = F_THRESHOLD;
        trackerData_event.readIntrinsicParameter(CAM_NAMES, 1);
        trackerData_event.f_threshold = F_THRESHOLD_1;
        if (FISHEYE) {
            trackerData_img.fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            trackerData_event.fisheye_mask = cv::imread(FISHEYE_MASK, 0);
        }
    }

    ~EventVisualizer() {
        if (this->estimator != NULL) delete this->estimator;
    }



    void create_eventpackage() {
        if (!new_image || time_buffer_.empty() || event_buffer_.empty()) {
            return;  // 无时间戳或事件可处理，直接返回
        }
        // 取出第一个pair（时间戳, 图像）
        auto cur_pair = time_buffer_.front();
        ros::Time current_time = cur_pair.first;
        cv::Mat current_image = cur_pair.second;
        // 找到当前时间戳之后的事件
        auto it = event_buffer_.begin();
        if (is_first_frame && it->ts > current_time) {
            ROS_WARN("No events found before timestamp %.6f, need Waiting for more events.", current_time.toSec());
            time_buffer_.pop_front();  // 移除已处理的时间戳
            return;
        }
        while (it != event_buffer_.end() && it->ts <= current_time) {
            ++it;
        }
        if (it == event_buffer_.end()) {
            ROS_WARN("No events found after timestamp %.6f", current_time.toSec());
            return;
        }
        EventArrayPtr event_array_ptr = std::make_shared<EventArray>();
        if (is_first_frame) {
            if (INITIAL_EVENT_COUNT > event_buffer_.size()) {
                ROS_WARN(" events available after timestamp %.6f, need Waiting for more events.", current_time.toSec());
                return;  // 事件不足，等待更多事件
            }
            for (ull count = 0; count < INITIAL_EVENT_COUNT && it != event_buffer_.end(); ++count, ++it) {
                event_array_ptr->push_back(*it);
            }
            is_first_frame = false;
        }else{
            ull remaining_event_time = event_buffer_.end()->ts.toNSec() - current_time.toNSec();
            size_t num_events = std::distance(it, event_buffer_.end());
            if (num_events*(remaining_event_time) < (previous_event_count * interval_time_ns)||(num_events<MIN_EVENT_COUNT)) {
                ROS_WARN(" events available after timestamp %.6f, need Waiting for more events.", current_time.toSec());
                return;  // 事件不足，等待更多事件
            }
            for (ull count = 0; count < MAX_EVENT_COUNT && count*((*it).ts.toNSec() - current_time.toNSec()) < (previous_event_count * interval_time_ns ) && it != event_buffer_.end(); ++count, ++it) {
                event_array_ptr->push_back(*it);
            }
            // 如果事件数小于下限，补足到下限（如果有足够事件）
            ull current_size = event_array_ptr->size();
            while (current_size < MIN_EVENT_COUNT && it != event_buffer_.end()) {
                event_array_ptr->push_back(*it);
                ++current_size;
                ++it;
            }
        }
        // 调用函数进行优化
        if (!event_array_ptr->empty()) {
            previous_event_count = event_array_ptr->size();
            // 传递当前图像指针
            image_cb(event_array_ptr, current_time, current_image);  // 处理事件帧
            time_buffer_.pop_front();  // 移除已处理的时间戳
            ROS_INFO("Processed timestamp %.6f with %zu events. Remaining buffer size: %zu", 
                    current_time.toSec(), event_array_ptr->size(), time_buffer_.size());
            if (!time_buffer_.empty()) {
                create_eventpackage();
            } else {
                new_image = false;
            }
        } else {
            ROS_WARN("No events found after timestamp %.6f.", current_time.toSec());
        }
    }


    
            
    void image_slice(const sensor_msgs::ImageConstPtr& msg) {
        // 将新时间戳和图像加入缓冲区
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "mono8");
            // 检查图像尺寸是否为346x260
            if (cv_ptr->image.cols != 346 || cv_ptr->image.rows != 260) {
                ROS_ERROR("图像尺寸不匹配: 当前帧 %dx%d, 期望尺寸 346x260", cv_ptr->image.cols, cv_ptr->image.rows);
                return; // 跳过处理该帧
            }

            double time0 = msg->header.stamp.toSec();
            // 首帧检测
            if (first_image_flag_) {
                first_image_flag_ = false;
                first_image_time_ = time0;
                last_image_time_ = time0;
                return;
            }
            // 跳变/倒退检测
            if (time0 - last_image_time_ > 1.0 || time0 < last_image_time_) {
                ROS_WARN("image discontinue! reset the event visualizer");
                first_image_flag_ = true;
                last_image_time_ = 0;
                pub_count_ = 1;
                std_msgs::Bool restart_flag;
                restart_flag.data = true;
                pub_restart_.publish(restart_flag);
                return;
            }
            last_image_time_ = time0;
            // 频率控制
            double time_interval = (time0 - first_image_time_);
            double pub_rate = (time_interval > 0) ? (1.0 * pub_count_ / time_interval) : 9999.0;
            bool pub_this_frame = (round(pub_rate) <= refresh_rate);
            if(pub_this_frame){
                if (pub_this_frame && fabs(pub_rate - refresh_rate) < 0.01 * refresh_rate) {
                    first_image_time_ = time0;
                    pub_count_ = 0;
                }
            } else {
                return; // 跳过本帧
            }
            
            // 所有检查通过后再插入
            time_buffer_.emplace_back(msg->header.stamp, cv_ptr->image.clone());
            ROS_INFO("Added image timestamp %.6f to buffer. Buffer size: %zu", msg->header.stamp.toSec(), time_buffer_.size());
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        // 如果当前没有正在处理的时间戳，触发处理
        if (!new_image) {
            new_image = true;
        }
    }

    void time_rate_slice(const dvs_msgs::EventArray::ConstPtr& msg){
        event_buffer_.insert(event_buffer_.end(), msg->events.begin(), msg->events.end());
        create_eventpackage();
        if (event_buffer_.size() > EVENT_HISTORY_SIZE)
        {
            size_t remove_events = event_buffer_.size() - EVENT_HISTORY_SIZE;
            event_buffer_.erase(event_buffer_.begin(), event_buffer_.begin()+remove_events);
        }
    }
    
    void image_cb(const EventArrayPtr event_array_ptr, ros::Time event_frame_time, const cv::Mat& current_image) {
        if (!event_array_ptr->empty()) {
            ros::Time first_event_time = (*event_array_ptr)[0].ts;
            ros::Time last_event_time = (*event_array_ptr)[event_array_ptr->size()-1].ts;
            // ROS_INFO("=== Event Package Info ===");
            // ROS_INFO("Event count: %zu", event_array_ptr->size());
            // ROS_INFO("First event timestamp: %.6f", first_event_time.toSec());
            // ROS_INFO("Last event timestamp: %.6f", last_event_time.toSec());
            // ROS_INFO("Event frame time: %.6f", event_frame_time.toSec());
            // ROS_INFO("=========================");
        }
        // ROS_INFO("event_array_ptr->size(): %zu", event_array_ptr->size());
        cv::Mat event_image;
        if (estimator != NULL) {
            for (int i = (event_array_ptr->size()-1); i >= 0; i--){
                ull time = (*event_array_ptr)[i].ts.toNSec();
                Event e((*event_array_ptr)[i].x, (*event_array_ptr)[i].y, time);
                estimator->add_event(e);
            }
            estimator->recompugit add src/VINS-Mono src/catkin_simple src/rpg_dvs_roste();
            // ROS_INFO("recompute done");
            cv::transpose(EventFile::projection_img(&estimator->ev_buffer, 1), event_image);
        }
        // 单独发布事件帧
        if (!event_image.empty()) {
            sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(std_msgs::Header(), "mono8", event_image).toImageMsg();
            msg0->header.stamp = event_frame_time;
            suppl_image_pub_0.publish(msg0);
        }
        // 融合current_image和event_image的特征点并发布
        trackerData_img.readImage(current_image, event_frame_time.toSec());

        if (!event_image.empty()) {
            trackerData_event.readImage(event_image, event_frame_time.toSec());
        }
        // 更新特征点 ID
        for (unsigned int i = 0;; i++) {
            bool completed = false;
            completed |= trackerData_event.updateID(i);
            if (!completed) break;
        }
        for (unsigned int i = 0;; i++) {
            bool completed = false;
            completed |= trackerData_img.updateID(i);
            if (!completed) break;
        }
        pub_count_++;

        // 构造融合特征点PointCloud
        sensor_msgs::PointCloudPtr merged_features(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        merged_features->header.stamp = event_frame_time;
        merged_features->header.frame_id = "world";

        // 添加current_image特征点（ID偶数）
        auto& un_pts0 = trackerData_img.cur_un_pts;
        auto& cur_pts0 = trackerData_img.cur_pts;
        auto& ids0 = trackerData_img.ids;
        auto& pts_velocity0 = trackerData_img.pts_velocity;
        for (unsigned int j = 0; j < ids0.size(); j++) {
            if (trackerData_img.track_cnt[j] > 1) {
                geometry_msgs::Point32 p;
                p.x = un_pts0[j].x;
                p.y = un_pts0[j].y;
                p.z = 1;
                merged_features->points.push_back(p);
                id_of_point.values.push_back(ids0[j]*2); // 偶数ID
                u_of_point.values.push_back(cur_pts0[j].x);
                v_of_point.values.push_back(cur_pts0[j].y);
                velocity_x_of_point.values.push_back(pts_velocity0[j].x);
                velocity_y_of_point.values.push_back(pts_velocity0[j].y);
            }
        }
        // 添加event_image特征点（ID奇数）
        auto& un_pts1 = trackerData_event.cur_un_pts;
        auto& cur_pts1 = trackerData_event.cur_pts;
        auto& ids1 = trackerData_event.ids;
        auto& pts_velocity1 = trackerData_event.pts_velocity;
        for (unsigned int j = 0; j < ids1.size(); j++) {
            if (trackerData_event.track_cnt[j] > 1) {
                geometry_msgs::Point32 p;
                p.x = un_pts1[j].x;
                p.y = un_pts1[j].y;
                p.z = 1;
                merged_features->points.push_back(p);
                id_of_point.values.push_back(ids1[j]*2+1); // 奇数ID
                u_of_point.values.push_back(cur_pts1[j].x);
                v_of_point.values.push_back(cur_pts1[j].y);
                velocity_x_of_point.values.push_back(pts_velocity1[j].x);
                velocity_y_of_point.values.push_back(pts_velocity1[j].y);
            }
        }
        merged_features->channels.push_back(id_of_point);
        merged_features->channels.push_back(u_of_point);
        merged_features->channels.push_back(v_of_point);
        merged_features->channels.push_back(velocity_x_of_point);
        merged_features->channels.push_back(velocity_y_of_point);
        if (init_pub) {
            init_pub = true;
        } else {
            pub_merged_.publish(merged_features);
        }
        // 输出特征点数量
        ROS_INFO("current_image特征点数: %zu", trackerData_img.cur_pts.size());
        ROS_INFO("event_image特征点数: %zu", trackerData_event.cur_pts.size());
        // 可视化特征点图像并发布
        // current_image
        if (!trackerData_img.cur_pts.empty()) {
            cv_bridge::CvImagePtr cv_ptr0(new cv_bridge::CvImage);
            cv_ptr0->encoding = sensor_msgs::image_encodings::BGR8;
            cv::Mat rgb_img0;
            cv::cvtColor(current_image, rgb_img0, CV_GRAY2RGB);
            for (unsigned int j = 0; j < trackerData_img.cur_pts.size(); j++) {
                double len = std::min(1.0, 1.0 * trackerData_img.track_cnt[j] / 20.0);
                cv::circle(rgb_img0, trackerData_img.cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }
            cv_ptr0->image = rgb_img0;
            cv_ptr0->header.stamp = event_frame_time;
            pub_match_0_.publish(cv_ptr0->toImageMsg());
        }
        // event_image
        if (!event_image.empty() && !trackerData_event.cur_pts.empty()) {
            cv_bridge::CvImagePtr cv_ptr1(new cv_bridge::CvImage);
            cv_ptr1->encoding = sensor_msgs::image_encodings::BGR8;
            cv::Mat rgb_img1;
            cv::cvtColor(event_image, rgb_img1, CV_GRAY2RGB);
            for (unsigned int j = 0; j < trackerData_event.cur_pts.size(); j++) {
                double len = std::min(1.0, 1.0 * trackerData_event.track_cnt[j] / 20.0);
                cv::circle(rgb_img1, trackerData_event.cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }
            cv_ptr1->image = rgb_img1;
            cv_ptr1->header.stamp = event_frame_time;
            pub_match_1_.publish(cv_ptr1->toImageMsg());
        }
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_tracker_event");
    ros::NodeHandle nh("~"); // 注意是波浪线，表示私有命名空间
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(nh);
    
    // topic names
    if (!nh.getParam("better_flow/input_event_topic", input_event_topic)) input_event_topic = "/dvs/events";
    if (!nh.getParam("better_flow/input_image_topic", input_image_topic)) input_image_topic = "/dvs/image_raw";
    if (!nh.getParam("better_flow/output_image_s0_topic", output_image_s0_topic)) output_image_s0_topic = "/dvs/event_image";
    
    // Refresh parameters
    if (!nh.getParam("better_flow/refresh_rate", refresh_rate)) refresh_rate = 15;
    if (!nh.getParam("better_flow/process_data", process_data)) process_data = true;
    ROS_INFO("process_data: %d", process_data);

    // Publisher
    ros::Publisher pub_merged = nh.advertise<sensor_msgs::PointCloud>("feature", 1000);
    ros::Publisher pub_match_0 = nh.advertise<sensor_msgs::Image>("feature_img_0", 1000);
    ros::Publisher pub_match_1 = nh.advertise<sensor_msgs::Image>("feature_img_1", 1000);
    ros::Publisher pub_restart = nh.advertise<std_msgs::Bool>("restart", 1000);

    // Compute refresh parameters
    float time_refresh = 1.0 / refresh_rate;
    ull event_refresh = LLONG_MAX;

    // Main class
    EventVisualizer<EVENT_WIDTH, FROM_SEC(TIME_WIDTH)> visualizer(
        nh, event_refresh, FROM_SEC(time_refresh),
        pub_merged, pub_match_0, pub_match_1, pub_restart);

    ros::spin();
    return 0;
};