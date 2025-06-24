#pragma once
#include <iomanip>
#include <numeric>

namespace math {
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

}  // namespace math