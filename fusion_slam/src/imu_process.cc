/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-09 23:02:09
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-15 23:47:05
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/imu_process.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "imu_process.hh"
#include <pcl/io/pcd_io.h>
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
    for (const auto& imu : sync_package.imu_queue_) {
        init_imu_count++;
        mean_acc += (imu.acc_ - mean_acc) / init_imu_count;
        mean_gyro += (imu.gyro_ - mean_gyro) / init_imu_count;
    }
    if (init_imu_count < 20) {
        init_success_ = false;
        return false;
    };

    LOG_INFO("mean acc:{},{},{}", mean_acc[0], mean_acc[1], mean_acc[2]);
    LOG_INFO("mean gyro:{},{},{}", mean_gyro[0], mean_gyro[1], mean_gyro[2]);
    // static_imu_init_->AddMeasurements(sync_package);
    // if (static_imu_init_->TryInit()) {
    //     LOG_INFO("TryInit");

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
        // mean_acc = static_imu_init_->GetInitBa();
        state.rot = Eigen::Quaterniond::FromTwoVectors((-mean_acc).normalized(), Eigen::Vector3d(0.0, 0.0, -1.0));
        state.grav = s2(Eigen::Vector3d(0, 0, -G_m_s2));
        LOG_INFO("state ror:{}", state.rot);
    } else {
        state.grav = s2(-mean_acc / mean_acc.norm() * G_m_s2);
    }
    LOG_INFO("gravity:{}", state.grav);
    // state.ba = mean_acc;
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
}

void IMUProcessor::PredictAndUndistort(MeasureGroup& sync_package, PointCloudPtr& undistort_pcl) {
    imus.clear();
    imus.push_back(last_imu_data_);
    imus.insert(imus.end(), sync_package.imu_queue_.begin(), sync_package.imu_queue_.end());
    const double lidar_begin_time = sync_package.lidar_begin_time;
    const double lidar_end_time = sync_package.lidar_end_time;
    const double imu_begin_time = imus.front().timestamped_;
    const double imu_end_time = imus.back().timestamped_;
    undistort_pcl = sync_package.current_lidar;
    // LOG_INFO("undistort before lidar size:{}", undistort_pcl->size());
    if (undistort_pcl->empty()) {
        return;
    }
    // sort
    std::sort(undistort_pcl->begin(), undistort_pcl->end(),
              [](const PointType& p1, const PointType& p2) { return p1.curvature < p2.curvature; });
    //  当前状态
    state_ikfom current_state = kf_->get_x();
    imu_poses_.clear();
    imu_poses_.emplace_back(0.0, last_acc, last_gyro, current_state.vel, current_state.pos,
                            current_state.rot.toRotationMatrix());
    V3D acc_mean, gyro_mean;
    double dt = 0.0;
    Q_.setIdentity();
    input_ikfom input;
    // 计算每一帧imu的位姿
    for (auto it = imus.begin(); it < (imus.end() - 1); it++) {
        IMUData& head = *it;
        IMUData& tail = *(it + 1);
        // lidar_being_time-imu_begin_time-imu_end_time---lidar_end_time;
        // 输入
        if (tail.timestamped_ < last_lidar_time_end_) {
            continue;
        }
        acc_mean = 0.5 * (head.acc_ + tail.acc_);
        // LOG_INFO("acc_mean:{}", acc_mean.transpose());
        gyro_mean = 0.5 * (head.gyro_ + tail.gyro_);
        // normalize acc ？
        acc_mean = acc_mean * G_m_s2 / mean_acc.norm();
        // LOG_INFO("norm acc_mean:{}", acc_mean.transpose());
        if (head.timestamped_ < last_lidar_time_end_) {
            dt = tail.timestamped_ - last_lidar_time_end_;
        } else {
            dt = tail.timestamped_ - head.timestamped_;
        }
        // LOG_INFO("DT:{}", dt);
        // 噪声矩阵
        Q_.block<3, 3>(0, 0).diagonal() = gyro_cov_;
        Q_.block<3, 3>(3, 3).diagonal() = acc_cov_;
        Q_.block<3, 3>(6, 6).diagonal() = gyro_bias_cov_;
        Q_.block<3, 3>(9, 9).diagonal() = acc_bias_cov_;
        input.gyro = gyro_mean;
        input.acc = acc_mean;
        // 预测
        kf_->predict(dt, Q_, input);
        current_state = kf_->get_x();
        // LOG_INFO("current_state:{}", current_state);
        last_gyro = gyro_mean - current_state.bg;
        last_acc = current_state.rot.toRotationMatrix() * (acc_mean - current_state.ba);
        last_acc += current_state.grav.get_vect();
        double offset = tail.timestamped_ - lidar_begin_time;
        imu_poses_.emplace_back(offset, last_acc, last_gyro, current_state.vel, current_state.pos,
                                current_state.rot.toRotationMatrix());
    }
    // 计算最后一个点云的位姿
    dt = lidar_end_time - imu_end_time;
    kf_->predict(dt, Q_, input);
    last_imu_data_ = imus.back();
    last_lidar_time_end_ = lidar_end_time;
    Undistort_cloud(undistort_pcl);
}

// 这里去畸变是转到帧尾的时刻
void IMUProcessor::Undistort_cloud(PointCloudPtr& out) {
    // pcl::io::savePCDFile("/home/hang/origin.pcd", *out);
    // 帧尾时刻的imu位姿
    state_ikfom end_state = kf_->get_x();
    M3D end_rot = end_state.rot.matrix();
    V3D end_pos = end_state.pos;
    V3D end_vel = end_state.vel;
    // lidar_to imu坐标系
    M3D r_il = end_state.offset_R_L_I.toRotationMatrix();
    V3D t_il = end_state.offset_T_L_I;

    auto it_pcl = out->points.end() - 1;
    double dt;
    for (auto it_kp = imu_poses_.end() - 1; it_kp != imu_poses_.begin(); --it_kp) {
        auto head = it_kp - 1;
        auto tail = it_kp;

        M3D imu_rot = head->rot;
        V3D imu_pos = head->pos;
        V3D imu_vel = head->vel;
        V3D imu_acc = tail->acc;
        V3D imu_gyro = tail->gyro;

        for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--) {
            dt = it_pcl->curvature / double(1000) - head->offset;
            V3D point_lidar(it_pcl->x, it_pcl->y, it_pcl->z);
            // 计算改点时刻的imu姿态
            M3D T_R_i = imu_rot * Sophus::SO3d::exp(imu_gyro * dt).matrix();
            V3D T_p_i = imu_pos + imu_vel * dt + 0.5 * imu_acc * dt * dt;
            //      p_l_end =  T_LI *   T_iendw * T_wi * T_IL * p_l
            Eigen::Vector3d p_compensate =
                r_il.transpose() *
                (end_rot.transpose() * (T_R_i * (r_il * point_lidar + t_il) + T_p_i - end_pos) - t_il);
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            if (it_pcl == out->points.begin()) break;
        }
    }
    // pcl::io::savePCDFile("/home/hang/undistort.pcd", *out);
}

void IMUProcessor::reset() {
    init_imu_count = 0;
    mean_acc = Eigen::Vector3d::Zero();
    mean_gyro = Eigen::Vector3d::Zero();
}

}  // namespace slam