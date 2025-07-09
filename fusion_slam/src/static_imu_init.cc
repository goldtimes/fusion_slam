#include "static_imu_init.hh"

namespace slam {
bool StaticImuInit::AddMeasurements(const MeasureGroup& measures) {
    if (measures.imu_queue_.empty()) {
        return false;
    }
    if (init_success_) {
        return true;
    }
    // 记录静止开始的时间
    if (init_imus_.empty()) {
        init_start_time = measures.imu_queue_.begin()->timestamped_;
    }
    init_imus_.insert(init_imus_.end(), measures.imu_queue_.begin(), measures.imu_queue_.end());
    double init_time = init_imus_.back().timestamped_ - init_start_time;
    if (init_time > options_.init_time_seconds) {
        // 允许初始化
        try_to_init_ = true;
    }
    while (init_imus_.size() > options_.init_buffer_size) {
        init_imus_.pop_front();
    }
    return true;
};

bool StaticImuInit::TryInit() {
    if (!try_to_init_) {
        return false;
    }
    // 这里开始初始化，计算加速度的均值，角速度的均值
    // 这里学习高翔的老师的用法
    Eigen::Vector3d mean_gyro, mean_acce;
    ComputeMeanAndCovDiag(init_imus_, mean_gyro, cov_gyro_, [](const IMUData& data) { return data.gyro_; });
    ComputeMeanAndCovDiag(init_imus_, mean_acce, cov_acc_, [](const IMUData& data) { return data.acc_; });
    LOG_INFO("mean acc:{},{},{}", mean_acce[0], mean_acce[1], mean_acce[2]);
    LOG_INFO("mean gyro:{},{},{}", mean_gyro[0], mean_gyro[1], mean_gyro[2]);
    // 估计重力后重新计算加速度的均值和方差
    gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;
    LOG_INFO("gravity:{},{},{}", gravity_[0], gravity_[1], gravity_[2]);
    ComputeMeanAndCovDiag(init_imus_, mean_acce, cov_acc_,
                          [this](const IMUData& data) { return data.acc_ + gravity_; });
    LOG_INFO("after add gravity mean acc:{},{},{}", mean_acce[0], mean_acce[1], mean_acce[2]);
    // 检查IMU噪声
    if (cov_gyro_.norm() > options_.max_static_gyro_var) {
        LOG_ERROR("陀螺仪测量噪声太大:{}, max_static_gyro_var:{}", cov_gyro_.norm(), options_.max_static_gyro_var);
        return false;
    }

    if (cov_acc_.norm() > options_.max_static_acc_var) {
        LOG_ERROR("加速度计测量噪声太大:{}, max_static_acc_var:{}", cov_acc_.norm(), options_.max_static_acc_var);
        return false;
    }
    // 估计测量噪声和零偏
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;
    last_imu_data = init_imus_.back();
    // 施密特正交来对齐重力
    LOG_INFO("Align Gravity");
    grad_schmit(mean_acce, R_GtoI);
    // LOG_INFO("R_GtoI:{}", R_GtoI);
    std::cout << "R_GtoI:\n" << R_GtoI << std::endl;
    init_success_ = true;
    return true;
}

void StaticImuInit::grad_schmit(const Eigen::Vector3d& gravity_inI, Eigen::Matrix3d& R_GtoI) {
    // 归一化z轴的方向
    Eigen::Vector3d z_axis = gravity_inI / gravity_inI.norm();
    Eigen::Vector3d x_axis, y_axis;
    Eigen::Vector3d e1(1, 0, 0);
    Eigen::Vector3d e2(0, 1, 0);
    double inner1 = e1.dot(z_axis) / z_axis.norm();
    double inner2 = e2.dot(z_axis) / z_axis.norm();
    if (fabs(inner1) < fabs(inner2)) {
        x_axis = z_axis.cross(e1);
        x_axis = x_axis / x_axis.norm();
        y_axis = z_axis.cross(x_axis);
        y_axis = y_axis / y_axis.norm();
    } else {
        x_axis = z_axis.cross(e2);
        x_axis = x_axis / x_axis.norm();
        y_axis = z_axis.cross(x_axis);
        y_axis = y_axis / y_axis.norm();
    }
    // Rotation from our global (where gravity is only along the z-axis) to the local one
    R_GtoI.block(0, 0, 3, 1) = x_axis;
    R_GtoI.block(0, 1, 3, 1) = y_axis;
    R_GtoI.block(0, 2, 3, 1) = z_axis;
}
}  // namespace slam