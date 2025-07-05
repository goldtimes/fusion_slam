/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-01 23:57:39
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-05 12:13:11
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/lidar_process.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEa
 */
#include "fastlio_odom/lidar_process.hh"
#include "common/logger.hpp"
#include "common_lib.hh"
#include "sensors/lidar.hh"

namespace slam::fastlio {
FastLidarProcess::FastLidarProcess(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf)
    : config_(config), ieskf_(ieskf) {
    LOG_INFO("FastLidarProcess init success!");
}

void FastLidarProcess::BuildLocalMap(const PointCloudPtr& cloud_world) {
}

void FastLidarProcess::Align(const PointCloudPtr& in_cloud) {
}
}  // namespace slam::fastlio