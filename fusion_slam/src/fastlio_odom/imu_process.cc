#include "fastlio_odom/imu_process.hh"
#include <memory>
#include "common_lib.hh"
#include "static_imu_init.hh"
namespace slam::fastlio {
IMUProcess::IMUProcess(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf)
    : config_(config), ieskf_(ieskf) {
    LOG_INFO("ImuProcess Start!");
    imu_init_ptr_ = std::make_shared<StateicImuInit>();
    Q_.setIdentity();
    // 高斯白噪声
    Q_.block<3, 3>(0, 0) = Mat3d::Identity() * config.ng;
    Q_.block<3, 3>(3, 3) = Mat3d::Identity() * config.na;
    // 零偏
    Q_.block<3, 3>(6, 6) = Mat3d::Identity() * config.nbg;
    Q_.block<3, 3>(9, 9) = Mat3d::Identity() * config.nba;
    last_acc_.setZero();
    last_gyro_.setZero();
}

bool IMUProcess::TrytoInit(const MeasureGroup& measure) {
    imu_init_ptr_->AddMeasurements(measure);
    if (imu_init_ptr_->TryInit()) {
        // 设置初始状态
        return true;
    } else {
        return false;
    }
}

void IMUProcess::PredictAndUndistort(const MeasureGroup& measure, PointCloud::Ptr& undistort_cloud) {
}

}  // namespace slam::fastlio