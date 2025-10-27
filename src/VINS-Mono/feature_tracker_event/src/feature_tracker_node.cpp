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

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;

// 输出提取的信息
ros::Publisher pub_merged, pub_match_0, pub_match_1;
ros::Publisher pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];

double first_image_time[NUM_OF_CAM];
int pub_count[NUM_OF_CAM] = {1, 1};
bool first_image_flag[NUM_OF_CAM] = {true, true};
double last_image_time[NUM_OF_CAM] = {0, 0};
bool init_pub[NUM_OF_CAM] = {false, false};

// 新增：控制是否显示跟踪的可视化标志（与第二个代码一致）
bool SHOW_TRACK_F = true;

void sync_callback(const sensor_msgs::ImageConstPtr& img0_msg, const sensor_msgs::ImageConstPtr& img1_msg) {
    // 获取时间戳
    double time0 = img0_msg->header.stamp.toSec();
    double time1 = img1_msg->header.stamp.toSec();

    // 将 ROS 图像消息转换为 OpenCV 格式
    cv_bridge::CvImageConstPtr ptr0 = cv_bridge::toCvCopy(img0_msg, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImageConstPtr ptr1 = cv_bridge::toCvCopy(img1_msg, sensor_msgs::image_encodings::MONO8);

    // 检查图像尺寸
    if (ptr0->image.cols != ptr1->image.cols || ptr0->image.rows != ptr1->image.rows) {
        ROS_ERROR("图像尺寸不匹配: 相机0 %dx%d, 相机1 %dx%d",
                  ptr0->image.cols, ptr0->image.rows, ptr1->image.cols, ptr1->image.rows);
        return; // 跳过处理该帧
    }

    // 验证 ROW 参数是否有效
    if (ROW > ptr0->image.rows) {
        ROS_ERROR("ROW (%d) 超出图像高度 (%d)", ROW, ptr0->image.rows);
        return; // 跳过处理该帧
    }

    // 判断时间戳是否相同（在阈值范围内）
    bool timestamps_match = (time0 == time1);

    // 相机0稳定性检测
    if (first_image_flag[0]) {
        first_image_flag[0] = false;
        first_image_time[0] = time0;
        last_image_time[0] = time0;
        return;
    }

    if (time0 - last_image_time[0] > 1.0 || time0 < last_image_time[0]) {
        ROS_WARN("image discontinue! reset the feature tracker for cam 0");
        first_image_flag[0] = true;
        last_image_time[0] = 0;
        pub_count[0] = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time[0] = time0;

    // 相机1稳定性检测
    if (first_image_flag[1]) {
        first_image_flag[1] = false;
        first_image_time[1] = time1;
        last_image_time[1] = time1;
        return;
    } 
    if (time1 - last_image_time[1] > 1.0 || time1 < last_image_time[1]) {
        ROS_WARN("image discontinue! reset the feature tracker for cam 1");
        first_image_flag[1] = true;
        last_image_time[1] = 0;
        pub_count[1] = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time[1] = time1;

    // 相机0发布频率控制
    double time_interval0 = (time0 - first_image_time[0]);
    double pub_rate0 = (time_interval0 > 0) ? (1.0 * pub_count[0] / time_interval0) : 9999.0;
    bool pub_this_frame0 = (round(pub_rate0) <= FREQ);
    if(pub_this_frame0){
        if (pub_this_frame0 && fabs(pub_rate0 - FREQ) < 0.01 * FREQ) {
            first_image_time[0] = time0;
            pub_count[0] = 0;
        }
    }

    trackerData[0].f_threshold = F_THRESHOLD;
    trackerData[1].f_threshold = F_THRESHOLD_1;

    // 将 ROS 图像消息转换为 OpenCV 格式并提取特征点
    trackerData[0].readImage(ptr0->image.rowRange(0, ROW), time0);
    trackerData[1].readImage(ptr1->image.rowRange(0, ROW), time1);

    // 更新特征点 ID
    for (unsigned int i = 0;; i++) {
        bool completed = false;
        completed |= trackerData[1].updateID(i);
        if (!completed) break;
    }
    for (unsigned int i = 0;; i++) {
        bool completed = false;
        completed |= trackerData[0].updateID(i);
        if (!completed) break;
    }

    // 仅当时间戳匹配且频率允许时发布融合特征点
    if (timestamps_match && pub_this_frame0) {
        pub_count[0]++;
        pub_count[1]++;

        sensor_msgs::PointCloudPtr merged_features(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;
        
        merged_features->header = img0_msg->header; // 使用相机0的时间戳
        merged_features->header.frame_id = "world";

        // 添加相机0的特征点
        auto& un_pts0 = trackerData[0].cur_un_pts;
        auto& cur_pts0 = trackerData[0].cur_pts;
        auto& ids0 = trackerData[0].ids;
        auto& pts_velocity0 = trackerData[0].pts_velocity;
        for (unsigned int j = 0; j < ids0.size(); j++) {
            if (trackerData[0].track_cnt[j] > 1) {
                geometry_msgs::Point32 p;
                p.x = un_pts0[j].x;
                p.y = un_pts0[j].y;
                p.z = 1;
                
                merged_features->points.push_back(p);
                id_of_point.values.push_back(ids0[j]*2); // 相机0的ID
                u_of_point.values.push_back(cur_pts0[j].x);
                v_of_point.values.push_back(cur_pts0[j].y);
                velocity_x_of_point.values.push_back(pts_velocity0[j].x);
                velocity_y_of_point.values.push_back(pts_velocity0[j].y);
            }
        }

        // 添加相机1的特征点（ID 偏移以区分）
        auto& un_pts1 = trackerData[1].cur_un_pts;
        auto& cur_pts1 = trackerData[1].cur_pts;
        auto& ids1 = trackerData[1].ids;
        auto& pts_velocity1 = trackerData[1].pts_velocity;
        for (unsigned int j = 0; j < ids1.size(); j++) {
            if (trackerData[1].track_cnt[j] > 1) {
                geometry_msgs::Point32 p;
                p.x = un_pts1[j].x;
                p.y = un_pts1[j].y;
                p.z = 1;
                merged_features->points.push_back(p);
                id_of_point.values.push_back(ids1[j]*2+1); // 相机1的ID偏移
                u_of_point.values.push_back(cur_pts1[j].x);
                v_of_point.values.push_back(cur_pts1[j].y);
                velocity_x_of_point.values.push_back(pts_velocity1[j].x);
                velocity_y_of_point.values.push_back(pts_velocity1[j].y);
            }
        }

        // 填充通道数据
        merged_features->channels.push_back(id_of_point);
        merged_features->channels.push_back(u_of_point);
        merged_features->channels.push_back(v_of_point);
        merged_features->channels.push_back(velocity_x_of_point);
        merged_features->channels.push_back(velocity_y_of_point);

        // 发布融合后的特征点
        if (!init_pub[1]) {
            init_pub[1] = true;
        } else {
            pub_merged.publish(merged_features);
        }

        // 新增：可视化功能（类似于第二个代码）
        if (SHOW_TRACK_F) {
            // 为相机0绘制特征点
            cv_bridge::CvImagePtr cv_ptr0 = cv_bridge::toCvCopy(img0_msg, sensor_msgs::image_encodings::MONO8);
            cv::Mat rgb_img0;
            cv::cvtColor(cv_ptr0->image, rgb_img0, CV_GRAY2RGB); // 转换为RGB
            for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++) {
                double len = std::min(1.0, 1.0 * trackerData[0].track_cnt[j] / WINDOW_SIZE);
                cv::circle(rgb_img0, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }
            // 更新cv_ptr0的图像和元数据
            cv_ptr0->image = rgb_img0;
            cv_ptr0->encoding = sensor_msgs::image_encodings::BGR8; // 更新编码为BGR8
            pub_match_0.publish(cv_ptr0->toImageMsg());

            // 为相机1绘制特征点
            cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(img1_msg, sensor_msgs::image_encodings::MONO8);
            cv::Mat rgb_img1;
            cv::cvtColor(cv_ptr1->image, rgb_img1, CV_GRAY2RGB);
            for (unsigned int j = 0; j < trackerData[1].cur_pts.size(); j++) {
                double len = std::min(1.0, 1.0 * trackerData[1].track_cnt[j] / WINDOW_SIZE);
                cv::circle(rgb_img1, trackerData[1].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }
            // 更新cv_ptr1的图像和元数据
            cv_ptr1->image = rgb_img1;
            cv_ptr1->encoding = sensor_msgs::image_encodings::BGR8;
            pub_match_1.publish(cv_ptr1->toImageMsg());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    ROS_INFO("this is main round of feature tracker");
    trackerData[0].f_threshold = F_THRESHOLD;    // 相机0
    trackerData[1].f_threshold = F_THRESHOLD_1;  // 相机1

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES, i);

    if (FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if (!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // 只保留 pub_merged 和可视化发布者
    pub_merged = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match_0 = n.advertise<sensor_msgs::Image>("feature_img_0", 1000);
    pub_match_1 = n.advertise<sensor_msgs::Image>("feature_img_1", 1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart", 1000);

    message_filters::Subscriber<sensor_msgs::Image> sub0_img(n, IMAGE0_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub1_img(n, IMAGE1_TOPIC, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub0_img, sub1_img); // 队列大小为10
    sync.registerCallback(boost::bind(&sync_callback, _1, _2));

    ros::spin();
    return 0;
}