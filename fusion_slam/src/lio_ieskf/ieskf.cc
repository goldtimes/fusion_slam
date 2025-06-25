#include "lio-ieskf/ieskf.hh"
#include "sophus/so3.hpp"

namespace slam {
void IESKF::BuildNoise(const Options& options) {
    double ev = options.acce_var_;
    double et = options.gyro_var_;
    double eg = options.bias_gyro_var_;
    double ea = options.bias_acce_var_;

    double ev2 = ev;  // * ev;
    double et2 = et;  // * et;
    double eg2 = eg;  // * eg;
    double ea2 = ea;  // * ea;

    // set Q
    Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

    double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
    double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
    double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
    gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
}

bool IESKF::Predict(const IMUData& last_imu, const IMUData& current_imu) {
    const double dt = current_imu.timestamped_ - last_imu.timestamped_;
    const double dt2 = dt * dt2;
    // 中值积分
    Eigen::Vector3d acc_mean = (last_imu.acc_ + current_imu.acc_) / 2;
    Eigen::Vector3d gyro_mean = (last_imu.gyro_ + current_imu.gyro_) / 2;

    Vec3d new_p = p_ + v_ * dt + 0.5 * (R_ * (acc_mean - ba_)) * dt2 + 0.5 * g_ * dt2;
    Vec3d new_v = v_ + (R_ * (acc_mean - ba_)) * dt + g_ * dt;
    SO3 new_R = R_ * Sophus::SO3d::exp((gyro_mean - bg_) * dt);

    R_ = new_R;
    v_ = new_v;
    p_ = new_p;

    // 协方差的传播
    Mat18d F = Mat18d::Identity();
    F.template block<3, 3>(0, 3) = Mat3d::Identity() * dt;
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(acc_mean - ba_) * dt;
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;
    F.template block<3, 3>(3, 15) = Mat3d::Identity() * dt;
    F.template block<3, 3>(6, 6) = SO3::exp(-(gyro_mean - bg_) * dt).matrix();
    F.template block<3, 3>(6, 9) = -Mat3d::Identity() * dt;

    cov_ = F * cov_ * F.transpose() + Q_;
    current_time_ = current_imu.timestamped_;
    return true;
}

// 迭代卡尔曼滤波更新
bool IESKF::UpdateUsingCustomObserve(CustomObsFunc obs) {
    SO3 start_R = R_;
    Eigen::Matrix<double, 18, 18> HTVH;
    Eigen::Matrix<double, 18, 1> HTVr;
    // 预测之后通过迭代卡尔曼滤波k次迭代后，得到k次迭代的状态
    Mat18d Qk, Pk;
    for (int iter = 0; iter < options_.num_iterations_; ++iter) {
        // 传入当前的状态量给点云，构建点面残差，构建大的观测矩阵
        obs(GetNominalSE3(), HTVH, HTVr);
        // 根据观测矩阵求得卡尔曼增益了来更新状态量
        // 投影P
        Mat18d J = Mat18d::Identity();
        J.template block<3, 3>(6, 6) = Mat3d::Identity() - 0.5 * SO3::hat((R_.inverse() * start_R).log());
        Pk = J * cov_ * J.transpose();

        // 卡尔曼更新
        Qk = (Pk.inverse() + HTVH).inverse();  // 这个记作中间变量，最后更新时可以用
        dx_ = Qk * HTVr;
        Update();
        // 迭代条件结束
        if (dx_.norm() < options_.quit_eps_) {
            break;
        }
    }
    // 结束迭代的时候更新协方差并且投影到切空间
    // update P
    cov_ = (Mat18d::Identity() - Qk * HTVH) * Pk;

    // project P
    Mat18d J = Mat18d::Identity();
    Vec3d dtheta = (R_.inverse() * start_R).log();
    J.template block<3, 3>(6, 6) = Mat3d::Identity() - 0.5 * SO3::hat(dtheta);
    cov_ = J * cov_ * J.inverse();

    dx_.setZero();
    return true;
}
}  // namespace slam