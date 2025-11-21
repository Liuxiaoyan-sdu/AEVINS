#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator.h"
#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;
//存储接收到的 IMU 数据。
std::condition_variable con;
//当前的时间戳
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
//存储接收到的特征点（图像）数据
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> event_feature_buf;
deque<geometry_msgs::Vector3StampedConstPtr> velocity_buf;
deque<nav_msgs::Odometry::ConstPtr>dvl_position_buf;

sensor_msgs::PointCloud fused_event_img_msg_buf;
//收到的重定位点云数据
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;
//保护数据缓冲区（imu_buf、feature_buf 和 relo_buf）的线程安全。
std::mutex m_buf;
//保护多线程同时改变变量时不产生竞争
std::mutex m_state;
//管理IMU数据
std::mutex i_buf;
//估计器是核心模块，多线程对其操作时需要加锁，确保数据一致性。
std::mutex m_estimator;
//最新的IMU时间戳？？？
double latest_time;
Eigen::Vector3d tmp_P;//位置信息
Eigen::Quaterniond tmp_Q;//姿态信息
Eigen::Vector3d tmp_V;//速度信息
Eigen::Vector3d tmp_Ba;//及速度偏执
Eigen::Vector3d tmp_Bg;//角速度偏执
Eigen::Vector3d acc_0;//加速度
Eigen::Vector3d gyr_0;//角速度
bool init_feature = 0;
bool init_event_feature = 0;
bool init_imu = 1;
//上次的IMU时间
double last_imu_t = 0;


// 目的：基于IMU数据进行状态预测，更新位置 (tmp_P)、姿态 (tmp_Q)、速度 (tmp_V)。

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);//计算姿态

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;//上一时刻的
    gyr_0 = angular_velocity;
}

// 目的：在状态更新时，将估计器的当前状态复制到临时变量，并对IMU缓冲区中的所有IMU数据进行预测。
// 复制IMU缓冲队列到临时缓冲区 tmp_imu_buf。
// 遍历临时缓冲区中的所有IMU消息，调用 predict 函数进行状态预测。
void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;//这次更新的第一次的IMU
    //对这一段进行不断的更新
    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

//插值代码DVL
geometry_msgs::Vector3StampedConstPtr interpolateDVL(const geometry_msgs::Vector3StampedConstPtr &v_before, const geometry_msgs::Vector3StampedConstPtr &v_after, double target_time)
{
    double t1 = v_before->header.stamp.toSec();
    double t2 = v_after->header.stamp.toSec();
    double ratio = (target_time - t1) / (t2 - t1);

    geometry_msgs::Vector3Stamped interpolated_msg;
    interpolated_msg.header.stamp = ros::Time(target_time);
    interpolated_msg.header.frame_id = v_before->header.frame_id;
    interpolated_msg.vector.x = v_before->vector.x + (v_after->vector.x - v_before->vector.x) * ratio;
    interpolated_msg.vector.y = v_before->vector.y + (v_after->vector.y - v_before->vector.y) * ratio;
    interpolated_msg.vector.z = v_before->vector.z + (v_after->vector.z - v_before->vector.z) * ratio;
    // ROS_INFO("DVL: before vx: %.2f, vy: %.2f, vz: %.2f", v_before->vector.x, v_before->vector.y, v_before->vector.z);
    // ROS_INFO("DVL: after vx: %.2f, vy: %.2f, vz: %.2f", v_after->vector.x, v_after->vector.y, v_after->vector.z);
    // ROS_INFO("DVL: target_time: %.2f", target_time);
    // ROS_INFO("DVL: t1: %.2f, t2: %.2f", t1, t2);
    // ROS_INFO("DVL: ratio: %.2f", ratio);
    // ROS_INFO("DVL: vx: %.2f, vy: %.2f, vz: %.2f", interpolated_msg.vector.x, interpolated_msg.vector.y, interpolated_msg.vector.z);

   return geometry_msgs::Vector3StampedConstPtr(new geometry_msgs::Vector3Stamped(interpolated_msg));

}

nav_msgs::Odometry::ConstPtr interpolateOdometry(const nav_msgs::Odometry::ConstPtr& odom_before, 
                                                 const nav_msgs::Odometry::ConstPtr& odom_after, 
                                                 double target_time) {
    // 提取时间戳
    double t1 = odom_before->header.stamp.toSec();
    double t2 = odom_after->header.stamp.toSec();
    
    // 计算插值比例
    double ratio = (target_time - t1) / (t2 - t1);

    // 创建插值后的 Odometry 消息
    nav_msgs::Odometry interpolated_msg;
    interpolated_msg.header.stamp = ros::Time(target_time);
    interpolated_msg.header.frame_id = odom_before->header.frame_id;
    interpolated_msg.child_frame_id = odom_before->child_frame_id;

    // 位置插值 (线性插值)
    interpolated_msg.pose.pose.position.x = odom_before->pose.pose.position.x + 
                                            (odom_after->pose.pose.position.x - odom_before->pose.pose.position.x) * ratio;
    interpolated_msg.pose.pose.position.y = odom_before->pose.pose.position.y + 
                                            (odom_after->pose.pose.position.y - odom_before->pose.pose.position.y) * ratio;
    interpolated_msg.pose.pose.position.z = odom_before->pose.pose.position.z + 
                                            (odom_after->pose.pose.position.z - odom_before->pose.pose.position.z) * ratio;

    // 姿态插值 (Slerp)
    Eigen::Quaterniond q_before(
        odom_before->pose.pose.orientation.w,
        odom_before->pose.pose.orientation.x,
        odom_before->pose.pose.orientation.y,
        odom_before->pose.pose.orientation.z
    );
    Eigen::Quaterniond q_after(
        odom_after->pose.pose.orientation.w,
        odom_after->pose.pose.orientation.x,
        odom_after->pose.pose.orientation.y,
        odom_after->pose.pose.orientation.z
    );
    
    // 使用 Eigen 的 Slerp 进行四元数插值
    Eigen::Quaterniond q_interpolated = q_before.slerp(ratio, q_after);

    // 将插值后的四元数赋值给消息
    interpolated_msg.pose.pose.orientation.w = q_interpolated.w();
    interpolated_msg.pose.pose.orientation.x = q_interpolated.x();
    interpolated_msg.pose.pose.orientation.y = q_interpolated.y();
    interpolated_msg.pose.pose.orientation.z = q_interpolated.z();

    // 返回智能指针
    return nav_msgs::Odometry::ConstPtr(new nav_msgs::Odometry(interpolated_msg));
}


//目的：从IMU和图像缓冲区中同步获取一组IMU数据和对应的图像数据，准备进行后续处理。
std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr, 
                       geometry_msgs::Vector3StampedConstPtr, nav_msgs::Odometry::ConstPtr>>
getMeasurements()
{
    
    
    std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr, 
                       geometry_msgs::Vector3StampedConstPtr, nav_msgs::Odometry::ConstPtr>> measurements;
    while (true)
    {
        //检查不同的模式下的缓冲区是否为空
        if (imu_buf.empty() || feature_buf.empty() || (velocity_buf.empty()&&DVL_ADD) || (dvl_position_buf.empty()&&DVL_POSE_ADD))
            return measurements;

        if(!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td)){
            sum_of_wait++;
            return measurements;
        }
        //在这里查找DVL的信息
        if(DVL_ADD){
            if(!(velocity_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td + TD_DVL)){
                return measurements;
            }
            if (!(velocity_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td + TD_DVL)){
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
            }
        }
        if(DVL_POSE_ADD){
            if(!(dvl_position_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td + TD_DVL)){
                return measurements;
            }
            if (!(dvl_position_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td + TD_DVL)){
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
            }
        }
        // 检查最早的IMU数据是否在图像数据之前，这个是比较慢的feature_buf普通帧的数据
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }

        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        
        feature_buf.pop();

        // 获取IMU数据
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        // 加入最后一帧IMU数据
        IMUs.emplace_back(imu_buf.front());

        geometry_msgs::Vector3StampedConstPtr velocity_msg ;
        if(DVL_ADD){
            while (velocity_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td + TD_DVL && velocity_buf[1]->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td + TD_DVL)
            {
               velocity_buf.pop_front();
            }
            velocity_msg = interpolateDVL(velocity_buf[0],velocity_buf[1],img_msg->header.stamp.toSec() + TD_DVL);
        }else{
            velocity_msg = nullptr;
        }
        nav_msgs::Odometry::ConstPtr position_msg ;
        if(DVL_POSE_ADD){
            while (dvl_position_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td + TD_DVL && dvl_position_buf[1]->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td + TD_DVL)
            {
               dvl_position_buf.pop_front();
            }
            position_msg = interpolateOdometry(dvl_position_buf[0],dvl_position_buf[1],img_msg->header.stamp.toSec() + TD_DVL);
        }else{
            position_msg = nullptr;
        }
        
        
        if (IMUs.empty())
            ROS_WARN("no imu between two image");

        // 将IMU数据、图像数据和多个事件数据并列存储, position_msg
        measurements.emplace_back(IMUs, img_msg , velocity_msg, position_msg);
    }
    return measurements;
}

// 目的：接收IMU消息并将其添加到IMU缓冲区，同时进行实时预测和发布最新的里程计（如果在非线性优化状态）。
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    // 使用互斥锁 m_buf 保护缓冲区，将IMU消息加入 imu_buf 队列
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
    //调用 predict 函数进行实时预测，更新临时状态变量。
    // 如果当前处于非线性优化状态（NON_LINEAR），发布最新的里程计信息（位置、姿态、速度）。
    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}
// const geometry_msgs::Vector3StampedConstPtr&
void dvl_callback(const geometry_msgs::Vector3StampedPtr& msg){
    // ROS_INFO("DVL: vx: %.2f, vy: %.2f, vz: %.2f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    // 线程安全地将数据存入缓冲区
    m_buf.lock();
    velocity_buf.push_back(msg);
    m_buf.unlock();
    // 通知等待的线程
    con.notify_one();
}

void dvl_position_callback(const nav_msgs::Odometry::ConstPtr& msg){
    // 输出当前时间戳和速度数据

    // 线程安全地将数据存入缓冲区
    m_buf.lock();
    dvl_position_buf.push_back(msg);
    m_buf.unlock();
    // 通知等待的线程
    con.notify_one();
}

// 目的：接收特征点（图像）消息并将其添加到图像缓冲区。
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    double current_time = feature_msg->header.stamp.toSec();
    // 输出当前时间戳
    ROS_INFO("feature timestamp: %f seconds", current_time);
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void event_feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    ROS_INFO("event feature callback!");
    if (!init_event_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_event_feature = 1;
        return;
    }
    double current_time = feature_msg->header.stamp.toSec();
    // 输出当前时间戳
    ROS_INFO("Event feature timestamp: %f seconds", current_time);
    m_buf.lock();
    event_feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("/////////////////////restart the estimator!//////////////////////");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}
// 目的：接收重定位（闭环）点云消息并将其添加到重定位缓冲区。
// 流程：
// 使用互斥锁 m_buf，将重定位消息加入 relo_buf 队列。
void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}
//目的：在独立的线程中处理同步后的IMU和图像数据，进行状态估计和优化，并发布结果。
//主要就在这个里面进行修改//3月19日
// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr, 
                       geometry_msgs::Vector3StampedConstPtr, nav_msgs::Odometry::ConstPtr>> measurements;
        // std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr, 
        //                geometry_msgs::Vector3StampedConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);//每次等同步完成
        //等待IMU同步
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        
        for (auto &measurement : measurements)
        {
            auto &imu_data = std::get<0>(measurement);  // 获取 IMU 数据
            auto &img_msg = std::get<1>(measurement);   // 获取图像数据
            auto &velocity_msg = std::get<2>(measurement);   // 获取速度数据
            auto &odometry_msg = std::get<3>(measurement);

            // auto &event_data = std::get<2>(measurement); // 获取事件数据
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : imu_data)//里边在此之前的所有IMU数据
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;//防止时间过早
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;//上一个时间
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else//对于
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;//上一时刻的为分x加上这一时刻的微分
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            //设置速度信息
            if(DVL_ADD){
                double vx = 0, vy = 0, vz = 0;
                vx =  velocity_msg->vector.x;
                vy =  velocity_msg->vector.y;
                vz =  velocity_msg->vector.z;
                // ROS_INFO("XXXDVL: vx: %.2f, vy: %.2f, vz: %.2f", vx, vy, vz);
                
                estimator.processDVL(Vector3d(vx, vy, vz));
            }

            if(DVL_POSE_ADD){
                // Declare variables for quaternion and position
                double qx = 0, qy = 0, qz = 0, qw = 0;  // Quaternion components
                double px = 0, py = 0, pz = 0;          // Position components
                // Extract quaternion from odometry_msg
                qx = odometry_msg->pose.pose.orientation.x;
                qy = odometry_msg->pose.pose.orientation.y;
                qz = odometry_msg->pose.pose.orientation.z;
                qw = odometry_msg->pose.pose.orientation.w;     
                // Extract position from odometry_msg
                px = odometry_msg->pose.pose.position.x;
                py = odometry_msg->pose.pose.position.y;
                pz = odometry_msg->pose.pose.position.z;
                // Create a 7D vector using Eigen (assuming VectorXd for dynamic size)
                Eigen::VectorXd pose_7d(7);
                pose_7d << px, py, pz, qx, qy, qz, qw;
                // Pass the 7D vector to the estimator (assuming it accepts VectorXd)
                estimator.processDVL_position(pose_7d);
            }  
            // set relocalizati on frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }
            //处理图片信息
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;//代表特征的编号
                int feature_id = v*2;
                int camera_id = 0;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;//化到单位深度
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];//像素坐标点
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];//光流的速度信息
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            ROS_INFO("0");
            estimator.processImage(image, img_msg->header);
            estimator.process(img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";
        
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();//进行
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_dvl = n.subscribe(DVL_TOPIC, 2000, dvl_callback);
    ros::Subscriber sub_dvl_pose = n.subscribe(DVL_POSITION_TOPIC, 2000, dvl_position_callback);
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // ros::Subscriber sub_image_1 = n.subscribe("/feature_tracker/feature_1", 2000, event_feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
