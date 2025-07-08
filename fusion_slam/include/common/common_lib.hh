#pragma once
#include <deque>
#include "eigen_type.hh"
#include "sensors/encoder.hh"
#include "sensors/gnss.hh"
#include "sensors/imu.hh"
#include "sensors/lidar.hh"

namespace slam {
struct MeasureGrounp {
    MeasureGrounp() {
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