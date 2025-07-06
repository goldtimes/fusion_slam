/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-05 00:15:43
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-07 00:24:56
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/fastlio_odom.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "fastlio_odom/fastlio_odom.hh"
#include <memory>
#include "common/pointcloud_utils.hh"
#include "common_lib.hh"
#include "fastlio_odom/imu_process.hh"
#include "fastlio_odom/lidar_process.hh"
#include "sensors/lidar.hh"

namespace slam::fastlio {

FastLioOdom::FastLioOdom(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf) : config_(config) {
    LOG_INFO("FastlioOdom start!");
    ieskf_ = ieskf;
    lidar_process_ptr_ = std::make_shared<FastLidarProcess>(config, ieskf);
    imu_process_ptr_ = std::make_shared<IMUProcess>(config, ieskf);
}

void FastLioOdom::ProcessSyncpackage(MeasureGroup& measure) {
    // 开始处理数据
    if (odom_state == ODOM_STATE::IMU_INIT) {
        // try to init
        if (imu_process_ptr_->TrytoInit(measure)) {
            LOG_INFO("imu init success!");
            odom_state = ODOM_STATE::MAP_INIT;
        }
        return;
    }
    // predict and undistort cloud;
    PointCloud::Ptr undistort(new PointCloud);
    imu_process_ptr_->PredictAndUndistort(measure, undistort);
    if (odom_state == ODOM_STATE::MAP_INIT) {
        // 第一帧,转到世界坐标系下
        LOG_INFO("MAP_INIT");
        auto cloud_world = TransformCloud(undistort, lidar_process_ptr_->GetRLtoG(), lidar_process_ptr_->GetTLtoG());
        lidar_process_ptr_->BuildLocalMap(cloud_world);
        odom_state = ODOM_STATE::MAPPING;
        return;
    }
    // 配准
    measure.curr_cloud = undistort;
    LOG_INFO("MAPPING");
    lidar_process_ptr_->Align(undistort);
}
}  // namespace slam::fastlio