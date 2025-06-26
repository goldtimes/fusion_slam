#pragma once
#include "eigen_type.hh"
#include "sophus/so3.hpp"

namespace slam {
struct NaviState {
    NaviState() = default;

    // from time, R, p, v, bg, bad
    explicit NaviState(double time, const SO3& R = SO3(), const Vec3d& t = Vec3d::Zero(),
                       const Vec3d& v = Vec3d::Zero(), const Vec3d& bg = Vec3d::Zero(), const Vec3d& ba = Vec3d::Zero())
        : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba) {
    }

    // from pose and vel
    NaviState(double time, const SE3& pose, const Vec3d& vel = Vec3d::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {
    }
    /// 转换到Sophus
    Sophus::SE3d GetSE3() const {
        return Sophus::SE3d(R_, p_);
    }

    friend std::ostream& operator<<(std::ostream& os, const NaviState& s) {
        os << "p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
           << ", q: " << s.R_.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg_.transpose()
           << ", ba: " << s.ba_.transpose();
        return os;
    }

    double timestamp_;
    Vec3d p_ = Vec3d::Zero();
    Vec3d v_ = Vec3d::Zero();
    SO3 R_;
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d gravity_ = Vec3d::Zero();
    Vec3d acc_ = Vec3d::Zero();
    Vec3d gyro_ = Vec3d::Zero();
};
}  // namespace slam