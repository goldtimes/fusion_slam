/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-19 23:18:17
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-06 17:22:08
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/sensors/lidar.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <pcl/point_cloud.h>
#include "common/lidar_point_type.hh"

// 统一的点云格式
namespace slam {
using PointType = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
}  // namespace slam