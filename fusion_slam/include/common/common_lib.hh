/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-08 23:14:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-16 00:43:22
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/common/common_lib.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <deque>
#include <numeric>
#include "eigen_type.hh"
#include "fastlio_odom/IKFoM_toolkit/esekfom/esekfom.hpp"
#include "sensors/encoder.hh"
#include "sensors/gnss.hh"
#include "sensors/imu.hh"
#include "sensors/lidar.hh"

namespace slam {

constexpr int NUM_MATCH_POINTS = 5;
constexpr double G_m_s2 = 9.81;

// kf的相关定义
using vect3 = MTK::vect<3, double>;
using so3 = MTK::SO3<double>;
using s2 = MTK::S2<double, 98090, 10000, 1>;
using vect1 = MTK::vect<1, double>;
using vect2 = MTK::vect<2, double>;
constexpr int PROCESS_NOISE_DOF = 12;

// 状态量的定义,p,r,r_il,p_il,v,bg,ba,g  23维 这里的g用了二维的
MTK_BUILD_MANIFOLD(state_ikfom, ((vect3, pos))((so3, rot))((so3, offset_R_L_I))((vect3, offset_T_L_I))((vect3, vel))(
                                    (vect3, bg))((vect3, ba))((s2, grav)));

//  imu传播的输入量 加速度和角速度
MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));

// ieskf的过程噪声
MTK_BUILD_MANIFOLD(process_noise_ikfom, ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

MTK::get_cov<process_noise_ikfom>::type process_noise_cov();

// kf的相关定义

struct MeasureGroup {
    MeasureGroup() {
        lidar_begin_time = -1;
        lidar_end_time = 1;
        imu_queue_.clear();
        current_lidar.reset(new PointCloud);
    }
    double lidar_begin_time;
    double lidar_end_time;
    std::deque<IMUData> imu_queue_;
    std::deque<GNSS> gps_quque_;
    std::deque<Encoder> encoder_queue_;
    PointCloudPtr current_lidar;
};

inline V3D PointToEigen(const PointType& point) {
    V3D point_eigen;
    point_eigen << point.x, point.y, point.z;
    return point_eigen;
}

inline PointType EigenToPoint(const V3D& point) {
    PointType p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    return p;
}

inline PointCloudPtr transformCloud(const PointCloudPtr& input_cloud, const Eigen::Matrix3d& rot, const V3D& trans) {
    PointCloudPtr out(new PointCloud);
    out->reserve(input_cloud->size());
    for (int i = 0; i < input_cloud->size(); ++i) {
        auto point_eigen = PointToEigen(input_cloud->points[i]);
        V3D trans_point_eigen = rot * point_eigen + trans;
        auto trans_point = EigenToPoint(trans_point_eigen);
        out->push_back(trans_point);
    }
    out->width = out->points.size();
    out->height = 1;  // 无组织点云
    out->is_dense = input_cloud->is_dense;
    return out;
}

inline double sq_dist(const PointType& p1, const PointType& p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

Eigen::Matrix<double, 24, 1> get_f(state_ikfom& s, const input_ikfom& in);
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom& s, const input_ikfom& in);
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom& s, const input_ikfom& in);

/**
    C 为容器的类型 队列，vector
    D 为均值的类型 Eigen::Vector3d
    Getter 为函数 所以这里是一个右值引用
    比如collects 为std::deque<IMUData>,data 就是IMUData, gettter函数就是获得里面的acc/gyro
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& collects, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = collects.size();
    // 计算均值
    mean = std::accumulate(collects.begin(), collects.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) /
           len;
    // 计算协方差对角线
    cov_diag = std::accumulate(collects.begin(), collects.end(), D::Zero().eval(),
                               [&getter, &mean](const D& sum, const auto& data) {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) /
               (len - 1);
}

template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C& collects, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov,
                       Getter&& getter) {
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = collects.size();
    // 计算均值
    // clang-format off

    mean = std::accumulate(collects.begin(), collects.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    // 计算协方差对角线
    cov = std::accumulate(collects.begin(), collects.end(), E::Zero().eval(),
                               [&getter, &mean](const E& sum, const auto& data) -> E{
                                auto value = getter(data).eval();
                                D v = value - mean;
                                return sum + v * v.transpose();
                               }) / (len - 1);
    // clang-format on
}
/**
    增量ndt
 */
template <typename S, int dim>
void UpdateMeanAndCov(int history_points, int added_points, const Eigen::Matrix<S, dim, 1>& olded_mean,
                      const Eigen::Matrix<S, dim, dim>& older_cov, const Eigen::Matrix<S, dim, 1>& added_mean,
                      const Eigen::Matrix<S, dim, dim>& added_cov, Eigen::Matrix<S, dim, 1>& new_mean,
                      Eigen::Matrix<S, dim, dim>& new_cov) {
    new_mean = (history_points * olded_mean + added_points * added_mean) / (history_points + added_points);
    new_cov = (history_points * (older_cov + (olded_mean - new_mean) * (olded_mean - new_mean).template transpose()) +
               added_points * (added_cov + (added_mean - new_mean) * (added_mean - new_mean).template transpose())) /
              (history_points + added_points);
}

bool esti_plane(Eigen::Vector4d& out, const PointVector& points, const double& thresh);
geometry_msgs::TransformStamped eigen2Transform(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos,
                                                const std::string& frame_id, const std::string& child_frame_id,
                                                const double& timestamp);

nav_msgs::Odometry eigen2Odometry(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos, const std::string& frame_id,
                                  const std::string& child_frame_id, const double& timestamp);

sensor_msgs::PointCloud2 pcl2msg(PointCloudPtr inp, std::string& frame_id, const double& timestamp);

}  // namespace slam