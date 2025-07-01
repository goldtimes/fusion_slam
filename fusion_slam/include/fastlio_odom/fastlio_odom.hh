#pragma once
#include <memory>
#include "fastlio_odom/imu_process.hh"
#include "fastlio_odom/lidar_process.hh"
namespace slam::fastlio {

class FastLioOdom {
   public:
    FastLioOdom();

   private:
    std::shared_ptr<IMUProcess> imu_process_ptr_;
    std::shared_ptr<LidarProcess> lidar_process_ptr_;
};
}  // namespace slam::fastlio