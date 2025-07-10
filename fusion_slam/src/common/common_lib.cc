/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-09 23:02:09
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-11 00:41:21
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/common/common_lib.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "common/common_lib.hh"
#include "pcl_conversions/pcl_conversions.h"

namespace slam {

MTK::get_cov<process_noise_ikfom>::type process_noise_cov() {
    MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
    MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);  // 0.03
    MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na,
                                                    0.0001);  // *dt 0.01 0.01 * dt * dt 0.05
    MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg,
                                                    0.00001);  // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
    MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba,
                                                    0.00001);  // 0.001 0.05 0.0001/out 0.01
    return cov;
}

Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    vect3 a_inertial = s.rot * (in.acc - s.ba);
    for (int i = 0; i < 3; i++) {
        res(i) = s.vel[i];
        res(i + 3) = omega[i];
        res(i + 12) = a_inertial[i] + s.grav[i];
    }
    return res;
}

Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
    cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
    vect3 acc_;
    in.acc.boxminus(acc_, s.ba);
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_);
    cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
    Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
    Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
    s.S2_Mx(grav_matrix, vec, 21);
    cov.template block<3, 2>(12, 21) = grav_matrix;
    cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
    return cov;
}

Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
    cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
    cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
    cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
    return cov;
}

bool esti_plane(Eigen::Vector4d &out, const PointVector &points, const double &thresh) {
    Eigen::Matrix<double, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<double, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0;
    for (int i = 0; i < NUM_MATCH_POINTS; i++) {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = points[i].z;
    }

    Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

    double norm = normvec.norm();
    out[0] = normvec(0) / norm;
    out[1] = normvec(1) / norm;
    out[2] = normvec(2) / norm;
    out[3] = 1.0 / norm;

    for (int j = 0; j < NUM_MATCH_POINTS; j++) {
        if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh) {
            return false;
        }
    }
    return true;
}
geometry_msgs::TransformStamped eigen2Transform(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos,
                                                const std::string &frame_id, const std::string &child_frame_id,
                                                const double &timestamp) {
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = frame_id;
    transform.header.stamp = ros::Time().fromSec(timestamp);
    transform.child_frame_id = child_frame_id;
    transform.transform.translation.x = pos(0);
    transform.transform.translation.y = pos(1);
    transform.transform.translation.z = pos(2);
    Eigen::Quaterniond q = Eigen::Quaterniond(rot);
    // std::cout << rot << std::endl;
    // std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    return transform;
}

nav_msgs::Odometry eigen2Odometry(const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos, const std::string &frame_id,
                                  const std::string &child_frame_id, const double &timestamp) {
    nav_msgs::Odometry odom;
    odom.header.frame_id = frame_id;
    odom.header.stamp = ros::Time().fromSec(timestamp);
    odom.child_frame_id = child_frame_id;
    Eigen::Quaterniond q = Eigen::Quaterniond(rot);
    odom.pose.pose.position.x = pos(0);
    odom.pose.pose.position.y = pos(1);
    odom.pose.pose.position.z = pos(2);

    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    return odom;
}

sensor_msgs::PointCloud2 pcl2msg(PointCloudPtr inp, std::string &frame_id, const double &timestamp) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*inp, msg);
    if (timestamp < 0)
        msg.header.stamp = ros::Time().now();
    else
        msg.header.stamp = ros::Time().fromSec(timestamp);
    msg.header.frame_id = frame_id;
    return msg;
}

}  // namespace slam