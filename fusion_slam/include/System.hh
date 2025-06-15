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
#include <memory>
#include "common/lidar_model.hh"
#include "ros/ros.h"
namespace slam {
class SystemConfig;
};

namespace slam {
class System {
   public:
    System(const ros::NodeHandle& nh);
    ~System() = default;

   private:
    void InitConfigAndPrint();

    void InitSubPub();

    void InitLidarModel();

    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    // void LivoxCallback();

   public:
   private:
    ros::NodeHandle nh_;
    // 不需要定义类的对象，因为是单例，程序的生命周期都存在
    // SystemConfig& sys_config;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber encoder_sub_;
};

}  // namespace slam
