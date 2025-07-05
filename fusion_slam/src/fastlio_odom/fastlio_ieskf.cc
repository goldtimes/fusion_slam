/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-02 23:27:47
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-05 15:35:22
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/fastlio_ieskf.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "common/eigen_type.hh"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "sophus/so3.hpp"

namespace slam::fastlio {

// 重写 + -
void NavState::operator+=(const V21D& delta) {
    R_ *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    P_ += delta.segment<3>(3);
    R_LtoI *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_LinI += delta.segment<3>(9);
    V_ += delta.segment<3>(12);
    bg_ += delta.segment<3>(15);
    ba_ += delta.segment<3>(18);
}
V21D NavState::operator-(const NavState& other) const {
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.R_.transpose() * R_).log();
    delta.segment<3>(3) = P_ - other.P_;
    delta.segment<3>(6) = Sophus::SO3d(other.R_LtoI.transpose() * R_LtoI).log();
    delta.segment<3>(9) = t_LinI - other.t_LinI;
    delta.segment<3>(12) = V_ - other.V_;
    delta.segment<3>(15) = bg_ - other.bg_;
    delta.segment<3>(18) = ba_ - other.ba_;
    return delta;
}
// 重写 <<
std::ostream& operator<<(std::ostream& os, const NavState& state) {
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.R_.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.P_.transpose() << std::endl;
    os << "r_il: " << state.R_LtoI.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_il: " << state.t_LinI.transpose() << std::endl;
    os << "v: " << state.V_.transpose() << std::endl;
    os << "bg: " << state.bg_.transpose() << std::endl;
    os << "ba: " << state.ba_.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}  // 设置初始化的旋转，外参，bg,ba,协方差噪声
void FastlioIESKF::SetInitState(const Mat3d& R_wi, const Mat3d& R_IL, const Vec3d& t_IL, const Vec3d& bg,
                                const Vec3d& ba, const Vec3d& gravity, const Vec3d& cov_acc, const Vec3d& cov_gyro) {
    state_.R_ = R_wi;
    state_.R_LtoI = R_IL;
    state_.t_LinI = t_IL;
    state_.bg_ = bg;
    state_.ba_ = ba;
    state_.g = gravity;
    cov_.setIdentity();
    // 设置外参的协方差
    cov_.block<3, 3>(6, 6) = Mat3d::Identity() * 0.0001;
    cov_.block<3, 3>(9, 9) = Mat3d::Identity() * 0.0001;
    // bg,ba
    cov_.block<3, 3>(15, 15).diagonal() = cov_gyro;
    cov_.block<3, 3>(18, 18).diagonal() = cov_acc;
}

void FastlioIESKF::Predict(const Vec3d& acc, const Vec3d& gyro, double dt, const M12D& Q) {
    // 状态和协方差的传播
    V21D delta_x = V21D::Zero();
    delta_x.segment<3>(0) = (gyro - state_.bg_) * dt;
    delta_x.segment<3>(3) = state_.V_ * dt;
    delta_x.segment<3>(12) = (state_.R_ * (acc - state_.ba_) + state_.g) * dt;
    F_.setIdentity();
    F_.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(gyro - state_.bg_) * dt).matrix();
    F_.block<3, 3>(0, 15) = -Sophus::SO3d::jr((gyro - state_.bg_) * dt) * dt;
    F_.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    F_.block<3, 3>(12, 0) = -state_.R_ * Sophus::SO3d::hat(acc - state_.ba_) * dt;
    F_.block<3, 3>(12, 18) = -state_.R_ * dt;

    G_.setZero();
    G_.block<3, 3>(0, 0) = -Sophus::SO3d::jr((gyro - state_.bg_) * dt) * dt;
    G_.block<3, 3>(12, 3) = -state_.R_ * dt;
    G_.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    G_.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    state_ += delta_x;
    cov_ = F_ * cov_ * F_.transpose() + G_ * Q * G_.transpose();
}
void FastlioIESKF::Update() {
    NavState predict_x = state_;
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    V21D delta = V21D::Zero();
    M21D H = M21D::Identity();
    V21D b;

    for (size_t i = 0; i < max_iter_; i++) {
        loss_func_(state_, shared_data);
        if (!shared_data.valid) break;
        H.setZero();
        b.setZero();
        delta = state_ - predict_x;
        M21D J = M21D::Identity();
        J.block<3, 3>(0, 0) = Sophus::SO3d::jr_inv(delta.segment<3>(0));
        J.block<3, 3>(6, 6) = Sophus::SO3d::jr_inv(delta.segment<3>(6));
        H += J.transpose() * cov_.inverse() * J;
        b += J.transpose() * cov_.inverse() * delta;

        H.block<12, 12>(0, 0) += shared_data.H;
        b.block<12, 1>(0, 0) += shared_data.b;

        delta = -H.inverse() * b;

        state_ += delta;
        shared_data.iter_num += 1;

        if (stop_func_(delta)) break;
    }

    M21D L = M21D::Identity();
    // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
    // L.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
    L.block<3, 3>(0, 0) = Sophus::SO3d::jr(delta.segment<3>(0));
    L.block<3, 3>(6, 6) = Sophus::SO3d::jr(delta.segment<3>(6));
    cov_ = L * H.inverse() * L.transpose();
}
}  // namespace slam::fastlio