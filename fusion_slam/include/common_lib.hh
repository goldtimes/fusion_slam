#pragma once

/**
    一些结构体的定义
 */

#include "sensors/imu.hh"
#include "sensors/encoder.hh"
#include "sensors/gnss.hh"
#include "sensors/lidar.hh"
#include <deque>

namespace slam {
    struct MeasureGroup{
        // lidar的帧头和帧尾时刻
        MeasureGroup(){
            lidar_begin_time = 0.0;
            curr_cloud.reset(new PointCloud());
        }

        double lidar_begin_time;
        double lidar_end_time;
        std::deque<IMUData> imus;
        std::deque<EncorderData> wheels;
        std::deque<GNSSData> gpss;
        PointCloudPtr curr_cloud;
    };
}