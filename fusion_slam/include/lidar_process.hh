#pragma once
#include "common/lidar_point_type.hh"
#include "SystemConfig.hh"
#include "common/lidar_model.hh"
#include <pcl/filters/voxel_grid.h>
#include "sensors/lidar.hh"
#include <sensor_msgs/PointCloud2.h>
#include "common/logger.hpp"
#include <pcl_conversions/pcl_conversions.h>
/**
    将标准的ros消息转换为自定义的点云类型
    如果使用loam的里程计则需要对点云进行特征提取
 */

namespace slam {
    class LidarProcess{
        public:
            LidarProcess();
            ~LidarProcess() = default;
        public:
            PointCloudPtr ConvertMessageToCloud(sensor_msgs::PointCloud2::ConstPtr& lidar_msg);
        private:
        // 体素滤波器
        pcl::VoxelGrid<PointXYZIRT> corner_voxel_filter_;
        pcl::VoxelGrid<PointXYZIRT> planer_voxel_filter_;
        // 特征提取类
    };

}