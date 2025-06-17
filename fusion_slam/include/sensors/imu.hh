#pragma  once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdint>

namespace slam {
    struct IMUData {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        uint64_t timestamped_; //us
        Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    };

}