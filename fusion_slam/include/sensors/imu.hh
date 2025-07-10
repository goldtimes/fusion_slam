#pragma once

#include <sensor_msgs/Imu.h>
#include "common/eigen_type.hh"

namespace slam {
struct IMUData {
    IMUData() {
    }
    IMUData(double time, const V3D& acc, const V3D& gyro) : timestamped_(time), acc_(acc), gyro_(gyro) {
    }
    IMUData(const sensor_msgs::Imu::ConstPtr& msg) {
        timestamped_ = msg->header.stamp.toSec();
        // if (msg->header.frame_id == "livox_frame") {
        //     acc_ << msg->linear_acceleration.x * 9.81, msg->linear_acceleration.y * 9.81,
        //         msg->linear_acceleration.z * 9.81;
        // } else {
        acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        // }
        gyro_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    }
    double timestamped_;
    V3D acc_ = V3D::Zero();
    V3D gyro_ = V3D::Zero();
};
}  // namespace slam