#pragma once
#include <Eigen/Core>
#include <cmath>
#include <iomanip>
#include <numeric>
#include "common/lidar_point_type.hh"

namespace math {
constexpr double kDEG2RAD = M_PI / 180.0;
constexpr double kRAD2DEG = 180.0 / M_PI;
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

// 实现模板的矢量比较
template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};
// 模板的片特化
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) || (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

/// 矢量哈希
template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943)) % 10000000);
}

template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
}

}  // namespace math