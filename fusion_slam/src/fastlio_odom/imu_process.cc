/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-09 23:02:09
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-10 00:37:51
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/imu_process.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "fastlio_odom/imu_process.hh"
#include "common/common_lib.hh"
#include "common/eigen_type.hh"
#include "static_imu_init.hh"

namespace slam {
IMUProcessor::IMUProcessor(const IMUProcessorConfig& config,
                           std::shared_ptr<esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>> kf)
    : config_(config), kf_(kf) {
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

void IMUProcessor::SetCov(double gyro_cov, double acc_cov, double gyro_bias_cov, double acc_bias_cov) {
    gyro_cov_ = Eigen::Vector3d(gyro_cov, gyro_cov, gyro_cov);
    acc_cov_ = Eigen::Vector3d(acc_cov, acc_cov, acc_cov);
    gyro_bias_cov_ = Eigen::Vector3d(gyro_bias_cov, gyro_bias_cov, gyro_bias_cov);
    acc_bias_cov_ = Eigen::Vector3d(acc_bias_cov, acc_bias_cov, acc_bias_cov);
}
void IMUProcessor::SetCov(const V3D& gyro_cov, const V3D& acc_cov, const V3D& gyro_bias_cov, const V3D& acc_bias_cov) {
    gyro_cov_ = gyro_cov;
    acc_cov_ = acc_cov;
    gyro_bias_cov_ = gyro_bias_cov;
    acc_bias_cov_ = acc_bias_cov;
}
bool IMUProcessor::TryInit(MeasureGroup& sync_package) {
    if (sync_package.imu_queue_.empty()) {
        return false;
    }
    static_imu_init_->AddMeasurements(sync_package);
    if (static_imu_init_->TryInit()) {
        LOG_INFO("TryInit");

        // 设置eskf的初始化状态
        // ieskf_->SetInitState(imu_init_ptr_->GetRGtoI(), config_.r_il, config_.t_il, imu_init_ptr_->GetinitBg(),
        //                      imu_init_ptr_->GetInitBa(), imu_init_ptr_->GetGravity(), imu_init_ptr_->GetCovAcc(),
        //                      imu_init_ptr_->GetCovGyro());
        // 当前状态量
        state_ikfom state = kf_->get_x();
        // 外参
        state.offset_R_L_I = rot_ext;
        state.offset_T_L_I = pos_ext;
        state.bg = mean_gyro;
        s2 grav;
        if (config_.align_gravity) {
            mean_acc = static_imu_init_->GetInitBa();
            auto rot = Eigen::Quaterniond::FromTwoVectors((-mean_acc).normalized(), Eigen::Vector3d(0.0, 0.0, -1.0));
            grav = s2(Eigen::Vector3d(0, 0, -G_m_s2));
            LOG_INFO("state ror:{}", rot.matrix().inverse() * -1);
        } else {
            grav = s2(-mean_acc / mean_acc.norm() * G_m_s2);
        }
        LOG_INFO("gravity:{}", grav);
        state.ba = static_imu_init_->GetInitBa();
        kf_->change_x(state);
        // 初始化协方差
        esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>::cov init_P = kf_->get_P();
        init_P.setIdentity();
        // 外参R
        init_P.block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
        // 外参t
        init_P.block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
        // bg
        init_P.block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
        // ba
        init_P.block<3, 3>(18, 18) = M3D::Identity() * 0.001;
        // g
        init_P.block<2, 2>(21, 21) = Eigen::Matrix<double, 2, 2>::Identity() * 0.00001;
        kf_->change_P(init_P);

        last_imu_data_ = sync_package.imu_queue_.back();
        last_lidar_time_end_ = sync_package.lidar_end_time;
        init_success_ = true;
        return true;
    } else {
        init_success_ = false;
        return false;
    }
}

void IMUProcessor::PredictAndUndistort(MeasureGroup& sync_package, PointCloudPtr& undistort_pcl) {
}
}  // namespace slam