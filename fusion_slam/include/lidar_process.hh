#pragma once

#include <livox_ros_driver2/CustomMsg.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensors/lidar.hh"

namespace slam {
enum class LIDAR_TYPE {
    AVIA = 0,
    MID360,
    ROBOSENSE,
    VELODYNE16,
    OUSTER64,
};
class LidarProcess {
   public:
    struct LidarProcessConfig {
        LidarProcessConfig() {
        }
        // 雷达类型
        LIDAR_TYPE lidar_type_ = LIDAR_TYPE::AVIA;
        // 线束
        int N_SCAN = 6;
        // 屏蔽的距离
        double blind = 0.2;
        // 过滤的点
        int point_filter_num = 2;
    };

   public:
    explicit LidarProcess(LidarProcessConfig config = LidarProcessConfig());
    ~LidarProcess() = default;
    void Process(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg, PointCloudPtr& pcl_out);
    void Process(const sensor_msgs::PointCloud2::ConstPtr& msg, PointCloudPtr& pcl_out);

   private:
    void avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr& msg);
    void robosen_handler(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void velo_handler(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void mid360_handler(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void ouster_handler(const sensor_msgs::PointCloud2::ConstPtr& msg);

   private:
    LidarProcessConfig config_;
    PointCloudPtr full_cloud_;
};
}  // namespace slam