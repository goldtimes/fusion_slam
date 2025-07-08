#pragma once
#include "common/eigen_type.hh"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
namespace slam {

struct GNSS {
    double timestamped_;
    V3D lla_;
};
}  // namespace slam