/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-08 23:14:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-09 00:10:50
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/common/common_lib.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <deque>
#include "eigen_type.hh"
#include "sensors/encoder.hh"
#include "sensors/gnss.hh"
#include "sensors/imu.hh"
#include "sensors/lidar.hh"

namespace slam {
struct MeasureGroup {
    MeasureGroup() {
        lidar_begin_time = -1;
        lidar_end_time = 1;
        imu_queue_.clear();
        current_lidar.reset(new PointCloud);
    }
    double lidar_begin_time;
    double lidar_end_time;
    std::deque<IMUData> imu_queue_;
    std::deque<GNSS> gps_quque_;
    std::deque<Encoder> encoder_queue_;
    PointCloudPtr current_lidar;
};
}  // namespace slam