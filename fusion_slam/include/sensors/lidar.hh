#pragma once
#include "common/lidar_point_type.hh"
#include <pcl/point_cloud.h>


// 统一的点云格式
namespace slam {
    using PointCloud = pcl::PointCloud<PointXYZIRT>;
    using PointCloudPtr = pcl::PointCloud<PointXYZIRT>::Ptr;
}