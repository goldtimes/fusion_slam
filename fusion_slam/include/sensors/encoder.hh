#pragma  once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdint>

namespace slam {
    struct EncorderData {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        uint64_t timestamped_; //us
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    };

}