/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-15 10:39:47
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-19 23:31:46
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/common/lidar_model.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "common/lidar_model.hh"
#include <mutex>
#include "common/logger.hpp"

namespace slam {

LidarModel* LidarModel::instance_ = nullptr;
std::string LidarModel::lidar_type_ = "";
LidarModel* LidarModel::GetInstance(const std::string lidar_type) {
    if (instance_ == nullptr) {
        static std::once_flag flag;
        lidar_type_ = lidar_type;
        std::call_once(flag, [&]() { instance_ = new LidarModel(lidar_type); });
    }
    return instance_;
}

LidarModel::LidarModel(const std::string& lidar_type) {
    if (lidar_type == "LeiShen_16") {
        vertical_scan_num_ = 16;
        horizon_scan_num_ = 2000;
        h_res_ = Degree2Radian(0.18f);
        v_res_ = Degree2Radian(2.0f);
        lower_angle_ = Degree2Radian(15.0f);
        lidar_sensor_type_ = LIDAR_TYPE::LEISHEN16;
    } else if (lidar_type == "RoboSense_16") {
        vertical_scan_num_ = 16;
        horizon_scan_num_ = 1800;
        h_res_ = Degree2Radian(0.2f);
        v_res_ = Degree2Radian(2.0f);
        lower_angle_ = Degree2Radian(15.0f);
        lidar_sensor_type_ = LIDAR_TYPE::ROBOSENSE16;
    } else if (lidar_type == "Velodyne_16") {
        vertical_scan_num_ = 16;
        horizon_scan_num_ = 1800;
        h_res_ = Degree2Radian(0.2f);
        v_res_ = Degree2Radian(2.0f);
        lower_angle_ = Degree2Radian(15.0f);
        lidar_sensor_type_ = LIDAR_TYPE::Velodyne;
    } else if (lidar_type == "Velodyne_32") {
        // 32线velodyne扫描头排列不均匀
        vertical_scan_num_ = 32;
        horizon_scan_num_ = 1800;
        h_res_ = Degree2Radian(0.2f);
        v_res_ = Degree2Radian(1.290322581f);
        lower_angle_ = Degree2Radian(30.0f);
        lidar_sensor_type_ = LIDAR_TYPE::Velodyne;
    } else if (lidar_type == "Velodyne_64") {
        vertical_scan_num_ = 64;
        horizon_scan_num_ = 1800;
        h_res_ = Degree2Radian(0.2f);
        v_res_ = Degree2Radian(0.4f);
        lower_angle_ = Degree2Radian(24.9f);
        lidar_sensor_type_ = LIDAR_TYPE::Velodyne;
    } else if (lidar_type == "Ouster_128_os1") {
        // 注意：该雷达的线号从上到下是0~127，并且当距离雷达很近时数据有异常，所以要滤波掉距离雷达近的点
        vertical_scan_num_ = 128;
        horizon_scan_num_ = 1024;
        h_res_ = Degree2Radian(360.0f / 1024.0f);
        v_res_ = Degree2Radian(0.35f);
        lower_angle_ = Degree2Radian(22.5f);
        lidar_sensor_type_ = LIDAR_TYPE::OUSTER128;
    } else if (lidar_type == "Livox_Mid_360") {
        vertical_scan_num_ = -1;
        horizon_scan_num_ = -1;
        h_res_ = 0.0f;
        v_res_ = 0.0f;
        lower_angle_ = 0.0f;
        lidar_sensor_type_ = LIDAR_TYPE::MID360;
    } else if (lidar_type == "Livox_Avia") {
        vertical_scan_num_ = -1;
        horizon_scan_num_ = -1;
        h_res_ = 0.0f;
        v_res_ = 0.0f;
        lower_angle_ = 0.0f;
        lidar_sensor_type_ = LIDAR_TYPE::AVIA;
    } else if (lidar_type == "None") {
        // when set lidar type to None, don't forget to set this following parameters
        vertical_scan_num_ = 0;
        horizon_scan_num_ = 0;
        h_res_ = 0.0f;
        v_res_ = 0.0f;
        lower_angle_ = 0.0f;
        lidar_sensor_type_ = LIDAR_TYPE::NONE;
        LOG_INFO(
            "You set lidar type as None, So don't forget to set lidar parameters by yourself. "
            "If you have set, can ignore this prompt");
    } else {
        LOG_INFO("Unsupported lidar sensor type");
    }
}
}  // namespace slam