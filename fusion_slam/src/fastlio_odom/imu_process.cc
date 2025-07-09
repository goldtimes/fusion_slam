#include "fastlio_odom/imu_process.hh"
#include "static_imu_init.hh"

namespace slam {
IMUProcessor::IMUProcessor(const IMUProcessorConfig& config,
                           std::shared_ptr<esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>> kf) {
    StaticImuInit::Options options;
    options.align_gravity = config.align_gravity;
    static_imu_init_ = std::make_shared<StaticImuInit>(options);
    // 这里初始化新的过程噪声矩阵
    Q_ = process_noise_cov();
}

void IMUProcessor::setExtParams(const M3D& rot, const V3D& pos) {
    rot_ext = rot;
    pos_ext = pos;
}
bool IMUProcessor::TryInit(MeasureGroup& sync_package) {
    if (sync_package.imu_queue_.empty()) {
        return false;
    }

    for (const auto& imu : sync_package.imu_queue_) {
        init_imu_count++;
        mean_acc += (imu.acc_ - mean_acc) / init_imu_count;
        mean_gyro += (imu.gyro_ - mean_gyro) / init_imu_count;
    }
    if (init_imu_count < 100) {
        return false;
    }
    init_success_ = true;
    LOG_INFO("mean acc:{},{},{}", mean_acc[0], mean_acc[1], mean_acc[2]);
    LOG_INFO("mean gyro:{},{},{}", mean_gyro[0], mean_gyro[1], mean_gyro[2]);
    // static_imu_init_->AddMeasurements(sync_package);
    // if (static_imu_init_->TryInit()) {
    // 设置eskf的初始化状态
    // ieskf_->SetInitState(imu_init_ptr_->GetRGtoI(), config_.r_il, config_.t_il, imu_init_ptr_->GetinitBg(),
    //                      imu_init_ptr_->GetInitBa(), imu_init_ptr_->GetGravity(), imu_init_ptr_->GetCovAcc(),
    //                      imu_init_ptr_->GetCovGyro());
    // 当前状态量
    // state_ikfom state = kf_->get_x();
    // 外参
    // state.offset_R_L_I = rot_ext;
    // state.offset_T_L_I = pos_ext;
    // state.bg = mean_gyro;
    // if (config_.align_gravity) {
    //     // mean_acc = static_imu_init_->GetInitBa();
    //     state.rot = Eigen::Quaterniond::FromTwoVectors((-mean_acc).normalized(), Eigen::Vector3d(0.0, 0.0, -1.0));
    //     state.grav = s2(Eigen::Vector3d(0, 0, -G_m_s2));
    //     std::cout << "state rot:\n" << state.rot << std::endl;
    // } else {
    //     state.grav = s2(-mean_acc / mean_acc.norm() * G_m_s2);
    // }
    // LOG_INFO("gravity:{},{},{}", state.grav);
    // // state.ba = static_imu_init_->GetInitBa();
    // kf_->change_x(state);
    // // 初始化协方差
    // esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>::cov init_P = kf_->get_P();
    // init_P.setIdentity();
    // // 外参R
    // init_P.block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    // // 外参t
    // init_P.block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    // // bg
    // init_P.block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    // // ba
    // init_P.block<3, 3>(18, 18) = M3D::Identity() * 0.001;
    // // g
    // init_P.block<2, 2>(21, 21) = Eigen::Matrix<double, 2, 2>::Identity() * 0.00001;
    // kf_->change_P(init_P);

    // last_imu_data_ = sync_package.imu_queue_.back();
    // last_lidar_time_end_ = sync_package.lidar_end_time;
    return true;
    // } else {
    //     return false;
    // }
}

void IMUProcessor::PredictAndUndistort(MeasureGroup& sync_package, PointCloudPtr& undistort_pcl) {
}
}  // namespace slam