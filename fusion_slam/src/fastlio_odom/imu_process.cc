/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-05 00:15:43
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-07 00:24:25
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/imu_process.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "fastlio_odom/imu_process.hh"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <memory>
#include <vector>
#include "common/eigen_type.hh"
#include "common_lib.hh"
#include "sensors/imu.hh"
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
        ieskf_->SetInitState(imu_init_ptr_->GetRGtoI(), config_.r_il, config_.t_il, imu_init_ptr_->GetinitBg(),
                             imu_init_ptr_->GetInitBa(), imu_init_ptr_->GetGravity(), imu_init_ptr_->GetCovAcc(),
                             imu_init_ptr_->GetCovGyro());
        last_imu_data_ = imu_init_ptr_->last_imu_data;
        m_last_propagate_end_time = measure.lidar_end_time;
        return true;
    } else {
        return false;
    }
}

void IMUProcess::PredictAndUndistort(const MeasureGroup& measure, PointCloud::Ptr& undistort_cloud) {
    undistort_cloud = measure.curr_cloud;
    LOG_INFO("cloud_size:{}", undistort_cloud->size());
    // save origin pcd
    // pcl::io::savePCDFile("/home/hang/origin.pcd", *undistort_cloud);
    std::vector<IMUData> imu_datas;
    imu_datas.push_back(last_imu_data_);
    imu_datas.insert(imu_datas.end(), measure.imus.begin(), measure.imus.end());
    imu_poses_.clear();
    auto state = ieskf_->GetState();
    imu_poses_.emplace_back(0.0, state.R_, state.P_, state.V_, last_acc_, last_gyro_);
    Vec3d acc_mean, gyro_mean;
    double dt;
    // 遍历所有的imu数据
    for (auto it_imu = imu_datas.begin(); it_imu < (imu_datas.end() - 1); ++it_imu) {
        auto&& head = *it_imu;
        auto&& tail = *(it_imu + 1);
        if (tail.timestamped_ < m_last_propagate_end_time) {
            continue;
        }
        gyro_mean = (tail.gyro_ + head.gyro_) * 0.5;
        acc_mean = (tail.acc_ + head.acc_) * 0.5;
        // 这里需要对时间戳进行处理
        if (head.timestamped_ < m_last_propagate_end_time) {
            dt = tail.timestamped_ - m_last_propagate_end_time;
        } else {
            dt = tail.timestamped_ - head.timestamped_;
        }
        // LOG_INFO("dt:{}", dt);
        ieskf_->Predict(acc_mean, gyro_mean, dt, Q_);
        state = ieskf_->GetState();
        // std::cout << "sate:" << state << std::endl;
        last_acc_ = state.R_ * (acc_mean - ieskf_->GetState().ba_) + state.g;
        last_gyro_ = gyro_mean - ieskf_->GetState().bg_;
        double offset = tail.timestamped_ - measure.lidar_begin_time;
        imu_poses_.emplace_back(0.0, state.R_, state.P_, state.V_, last_acc_, last_gyro_);
    }
    // 处理最后一个数据
    dt = measure.lidar_end_time - imu_datas.back().timestamped_;
    ieskf_->Predict(acc_mean, gyro_mean, dt, Q_);
    last_imu_data_ = imu_datas.back();
    m_last_propagate_end_time = measure.lidar_end_time;
    auto it_pcl = undistort_cloud->points.end() - 1;
    // 去畸变
    Mat3d cur_r_wi = ieskf_->GetState().R_;
    Vec3d cur_t_wi = ieskf_->GetState().P_;
    Mat3d cur_r_il = ieskf_->GetState().R_LtoI;
    Vec3d cur_t_il = ieskf_->GetState().t_LinI;

    auto undistort_start = std::chrono::high_resolution_clock::now();
    for (auto it_kp = imu_poses_.end() - 1; it_kp != imu_poses_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;

        Mat3d imu_r_wi = head->rot;
        Vec3d imu_t_wi = head->pos;
        Vec3d imu_vel = head->vel;
        Vec3d imu_acc = tail->acc;
        Vec3d imu_gyro = tail->gyro;

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            dt = it_pcl->curvature / double(1000) - head->offset_time;
            Vec3d point(it_pcl->x, it_pcl->y, it_pcl->z);
            Mat3d point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
            Vec3d point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
            Vec3d p_compensate =
                cur_r_il.transpose() *
                (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il);
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);
            if (it_pcl == undistort_cloud->points.begin()) break;
        }
    }
    auto undistort_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(undistort_end - undistort_start);
    LOG_INFO("undistort used time:{}, after undistort cloud size:{}", duration.count() * 1e-6, undistort_cloud->size());
    // pcl::io::savePCDFile("/home/hang/undistort.pcd", *undistort_cloud);
}

}  // namespace slam::fastlio