#pragma once

/**
    一些结构体的定义
 */

#include <deque>
#include "common/eigen_type.hh"
#include "sensors/encoder.hh"
#include "sensors/gnss.hh"
#include "sensors/imu.hh"
#include "sensors/lidar.hh"

namespace slam {

struct LIONodeConfig {
    std::string imu_topic;
    std::string lidar_topic;
    bool livox_msg;
    std::string body_frame;
    std::string world_frame;
    bool print_time_cost;

    int lidar_filter_num = 3;
    double lidar_min_range = 0.5;
    double lidar_max_range = 20.0;
    double scan_resolution = 0.15;
    double map_resolution = 0.3;

    double cube_len = 300;
    double det_range = 60;
    double move_thresh = 1.5;

    double na = 0.01;
    double ng = 0.01;
    double nba = 0.0001;
    double nbg = 0.0001;
    int imu_init_num = 20;
    int near_search_num = 5;
    int ieskf_max_iter = 5;
    bool gravity_align = true;
    bool esti_il = false;
    Mat3d r_il = Mat3d::Identity();
    Vec3d t_il = Vec3d::Zero();

    double lidar_cov_inv = 1000.0;
};
struct MeasureGroup {
    // lidar的帧头和帧尾时刻
    MeasureGroup() {
        lidar_begin_time = 0.0;
        curr_cloud.reset(new PointCloud());
    }

    double lidar_begin_time;
    double lidar_end_time;
    std::deque<IMUData> imus;
    std::deque<EncorderData> wheels;
    std::deque<GNSSData> gpss;
    PointCloudPtr curr_cloud;
};
}  // namespace slam