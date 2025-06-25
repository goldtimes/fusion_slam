/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-24 23:36:17
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-25 00:40:04
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/lio-ieskf/ieskf.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEam
 */
#pragma once

/**
   高翔老师的ieskf
 */
#include <functional>
#include "common/eigen_type.hh"
#include "common/navi_state.hh"
#include "math_utils.hh"
#include "sensors/imu.hh"
#include "sophus/so3.hpp"

namespace slam {
class IESKF {
   public:
    using SO3 = Sophus::SO3d;                           // 旋转变量类型
    using Vec3d = Eigen::Vector3d;                      // 向量类型
    using Vec18d = Eigen::Matrix<double, 18, 1>;        // 18维向量类型
    using Mat3d = Eigen::Matrix<double, 3, 3>;          // 3x3矩阵类型
    using MotionNoise = Eigen::Matrix<double, 18, 18>;  // 运动噪声类型
    using OdomNoise = Eigen::Matrix<double, 3, 3>;      // 里程计噪声类型
    using GnssNoise = Eigen::Matrix<double, 6, 6>;      // GNSS噪声类型
    using Mat18d = Eigen::Matrix<double, 18, 18>;       // 18维方差类型

    struct Options {
        Options() {
        }
        /// IEKF配置
        int num_iterations_ = 3;  // 迭代次数
        double quit_eps_ = 1e-3;  // 终止迭代的dx大小

        /// IMU 测量与零偏参数
        double imu_dt_ = 0.01;         // IMU测量间隔
        double gyro_var_ = 1e-5;       // 陀螺测量标准差
        double acce_var_ = 1e-2;       // 加计测量标准差
        double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

        /// RTK 观测参数
        double gnss_pos_noise_ = 0.1;                   // GNSS位置噪声
        double gnss_height_noise_ = 0.1;                // GNSS高度噪声
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // GNSS旋转噪声

        /// 其他配置
        bool update_bias_gyro_ = true;  // 是否更新bias
        bool update_bias_acce_ = true;  // 是否更新bias
    };

    IESKF(Options options = Options()) : options_(options) {
        BuildNoise(options);
    }

    IESKF(Options options, const Vec3d& init_bg, const Vec3d& init_ba, const Vec3d& gravity = Vec3d(0, 0, -9.8))
        : options_(options) {
        BuildNoise(options);
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;
    }

    /// 设置初始条件
    void SetInitialConditions(Options options, const Vec3d& init_bg, const Vec3d& init_ba,
                              const Vec3d& gravity = Vec3d(0, 0, -9.8)) {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;

        cov_ = 1e-4 * Mat18d::Identity();
        // 设置旋转的协方差
        cov_.block<3, 3>(6, 6) = 0.1 * math::kDEG2RAD * Mat3d::Identity();
    }

    bool Predict(const IMUData& last_imu, const IMUData& current_imu);
    bool Predict(const double& dt, const Eigen::Vector3d& acc_mean, const Eigen::Vector3d& gyro_mean);
    // 点云的配准得到的观测矩阵和卡尔曼滤波中的矩阵相乘
    using CustomObsFunc = std::function<void(const SE3& pose, Eigen::Matrix<double, 18, 18>& HT_inv_H,
                                             Eigen::Matrix<double, 18, 1>& HT_Vinv_r)>;

    bool UpdateUsingCustomObserve(CustomObsFunc obs);

    /// accessors
    /// 全量状态
    NaviState GetNominalState() const {
        return NaviState(current_time_, R_, p_, v_, bg_, ba_);
    }

    /// SE3 状态
    SE3 GetNominalSE3() const {
        return SE3(R_, p_);
    }
    void SetX(const NaviState& x) {
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
    }
    void SetCov(const Mat18d& cov) {
        cov_ = cov;
    }
    Vec3d GetGravity() const {
        return g_;
    }

   private:
    void BuildNoise(const Options& options);
    void Update() {
        p_ += dx_.block<3, 1>(0, 0);
        v_ += dx_.block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.block<3, 1>(6, 0));

        if (options_.update_bias_gyro_) {
            bg_ += dx_.block<3, 1>(9, 0);
        }

        if (options_.update_bias_acce_) {
            ba_ += dx_.block<3, 1>(12, 0);
        }
        g_ += dx_.block<3, 1>(15, 0);
    }

   private:
    Options options_;
    // 更新量
    Vec18d dx_ = Vec18d::Zero();
    //  协方差
    Mat18d cov_ = Mat18d::Zero();
    //  状态量
    double current_time_;
    SO3 R_;
    Vec3d p_ = Vec3d::Zero();
    Vec3d v_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d g_ = Vec3d::Zero();
    // 噪声项
    MotionNoise Q_ = MotionNoise::Zero();
    GnssNoise gnss_noise_ = GnssNoise::Zero();
};
}  // namespace slam