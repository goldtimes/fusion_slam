/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-14 02:39:34
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-16 00:22:28
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/Sytem.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%
 */
#pragma once

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <memory>
#include "common/lidar_model.hh"
#include <deque>
#include "sensor_msgs/NavSatFix.h"
#include "sensors/imu.hh"
#include "sensors/lidar.hh"
#include "sensors/encoder.hh"
#include "sensors/gnss.hh"
#include "lidar_process.hh"
#include <nav_msgs/Odometry.h>
#include "common_lib.hh"


namespace slam {
class SystemConfig;
class LidarProcess;
};

namespace slam {
class System {
   public:
    System(const ros::NodeHandle& nh);
    ~System() = default;

    void run();

   private:
    void InitConfigAndPrint();

    void InitSubPub();

    void InitLidarModel();

    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msgs);
    void GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msgs);
    // void LivoxCallback();

    bool sync_package(MeasureGroup& measure);
    

    void rosIMUtoIMU(const sensor_msgs::Imu::ConstPtr& imu_msgs, IMUData& imu_data, bool is_livox=false, bool has_orientation = false){
            imu_data.timestamped_ = static_cast<uint64_t>(imu_msgs->header.stamp.toSec() * 1e6);
            if (is_livox){
                const double gravity = 9.81;
                imu_data.acc_ = Eigen::Vector3d(imu_msgs->linear_acceleration.x * 9.81, imu_msgs->linear_acceleration.y * 9.81,imu_msgs->linear_acceleration.z * 9.81);
            }
            imu_data.gyro_ = Eigen::Vector3d(imu_msgs->angular_velocity.x, imu_msgs->angular_velocity.y, imu_msgs->angular_velocity.z);
            if (has_orientation){
                imu_data.orientation.w() = imu_msgs->orientation.w;
                imu_data.orientation.x() = imu_msgs->orientation.x;
                imu_data.orientation.y() = imu_msgs->orientation.y;
                imu_data.orientation.z() = imu_msgs->orientation.z;
            }
    }

   public:
   private:
    ros::NodeHandle nh_;
    std::shared_ptr<LidarProcess> lidar_process_;
    // 不需要定义类的对象，因为是单例，程序的生命周期都存在
    // SystemConfig& sys_config;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber encoder_sub_;
    std::deque<IMUData> imu_queue_;
    std::deque<PointCloudPtr> lidar_queue_;
    std::deque<double> lidar_time_queue_;
    std::deque<GNSSData> gnss_queue_;
    std::deque<EncorderData> encorder_queue_;

    bool use_odom_{false};
    bool use_gnss_{false};

    bool opt_with_odom{true};
    bool opt_with_gnss{true};
    
    bool process_lidar_{false};

    // 统计平均时间
    double lidar_mean_scantime_ = 0.0;
    uint64_t scan_num_ = 0;


    double last_lidar_timestamped_ = -1;
    double last_imu_timestamped_ = -1;
    bool imu_inited_ = false;

    MeasureGroup measure_;

};

}  // namespace slam
