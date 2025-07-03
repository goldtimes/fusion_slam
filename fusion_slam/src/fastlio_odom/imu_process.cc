#include "fastlio_odom/imu_process.hh"
#include <memory>
#include "common_lib.hh"
#include "static_imu_init.hh"
namespace slam::fastlio {
IMUProcess::IMUProcess(std::shared_ptr<FastlioIESKF> ieskf, const LIONodeConfig& config)
    : config_(config), ieskf_(ieskf) {
    LOG_INFO("ImuProcess Start!");
    imu_init_ptr_ = std::make_shared<StateicImuInit>();
}
}  // namespace slam::fastlio