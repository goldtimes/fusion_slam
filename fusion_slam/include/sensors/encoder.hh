#pragma once
#include "common/eigen_type.hh"
#include "nav_msgs/Odometry.h"
namespace slam {
struct Encoder {
    double timestamped_;
    V3D vel;
};
}  // namespace slam