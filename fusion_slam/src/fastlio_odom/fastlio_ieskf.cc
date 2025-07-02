#include "fastlio_odom/fastkio_ieskf.hh"

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
Vec3d NavState::operator-(const NavState& other) const {
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
    os << "t_il: " << state.P_.transpose() << std::endl;
    os << "r_il: " << state.R_LtoI.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_LinI.transpose() << std::endl;
    os << "v: " << state.V_.transpose() << std::endl;
    os << "bg: " << state.bg_.transpose() << std::endl;
    os << "ba: " << state.ba_.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void FastlioIESKF::Predict(const Vec3d& acc_mean, const Vec3d& gyro_mean, double dt, const M12D& Q) {
    // 状态和协方差的传播
}
void FastlioIESKF::Update() {
}
}  // namespace slam::fastlio