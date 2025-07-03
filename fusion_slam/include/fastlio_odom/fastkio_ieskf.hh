/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-02 00:10:49
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-02 00:14:36
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/fastlio_odom/fastkio_ieskf.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <Eigen/Core>
#include <functional>
#include <ostream>
#include <sophus/so3.hpp>
#include "common/eigen_type.hh"

namespace slam::fastlio {

// 对位姿和外参的求导
using M12D = Eigen::Matrix<double, 12, 12>;
// 21为r,t,r_li,t_li, v, bg, ba的状态量
using M21D = Eigen::Matrix<double, 21, 21>;

using V12D = Eigen::Matrix<double, 12, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;

struct SharedState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M12D H;
    V12D b;
    double res = 1e10;
    bool valid = false;
    size_t iter_num = 0;
};

struct NavState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Mat3d R_ = Mat3d::Identity();
    Vec3d P_ = Vec3d::Zero();
    Mat3d R_LtoI = Mat3d::Identity();
    Vec3d t_LinI = Vec3d::Zero();
    Vec3d V_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d g = Vec3d(0.0, 0.0, -9.81);
    // 重写 + -
    void operator+=(const V21D& dx);
    V21D operator-(const NavState& other) const;
    // 重写 <<
    friend std::ostream& operator<<(std::ostream& os, const NavState& state);
};
using LossFunc = std::function<void(NavState&, SharedState&)>;
using StopFunc = std::function<bool(const V21D&)>;
class FastlioIESKF {
   public:
    FastlioIESKF() = default;

    void SetLossFunc(LossFunc loss_func) {
        loss_func_ = loss_func;
    }
    void SetStopFunc(StopFunc stop_func) {
        stop_func_ = stop_func;
    }
    void Predict(const Vec3d& acc_mean, const Vec3d& gyro_mean, double dt, const M12D& Q);
    void Update();

    NavState& GetState() {
        return state_;
    }

    M21D& GetCov() {
        return cov_;
    }

   private:
    size_t max_iter_ = 3;
    // 状态量
    NavState state_;
    // 协方差
    M21D cov_;
    LossFunc loss_func_;
    StopFunc stop_func_;
    M21D F_;
    Eigen::Matrix<double, 21, 12> G_;
};
}  // namespace slam::fastlio