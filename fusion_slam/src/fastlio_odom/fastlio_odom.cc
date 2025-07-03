#include "fastlio_odom/fastlio_odom.hh"
#include <memory>
#include "common_lib.hh"
#include "fastlio_odom/imu_process.hh"
#include "fastlio_odom/lidar_process.hh"

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
        if (true) {
            odom_state = ODOM_STATE::MAP_INIT;
        }
        return;
    }
    // undistort cloud;
    if (odom_state == ODOM_STATE::MAP_INIT) {
        // 第一帧
        return;
    }
    // 配准
}
}  // namespace slam::fastlio