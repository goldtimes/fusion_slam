#pragma  once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdint>

namespace slam {
    struct GNSSData {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        uint64_t timestamped_; //us
        Eigen::Vector3d lla = Eigen::Vector3d::Zero();
        Eigen::Vector3d local_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d local_xyz_velocity = Eigen::Vector3d::Zero();
        Eigen::Matrix3d local_orientatin = Eigen::Matrix3d::Zero();
    };

}