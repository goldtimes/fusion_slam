/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-19 23:18:17
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-19 23:33:21
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/common/pointcloud_utils.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <execution>
#include "pcl/point_cloud.h"
#include "sensors/lidar.hh"

namespace slam {
template <typename PointT>
inline void RemoveNanFromPointCloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out) {
    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
        cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
        cloud_out.sensor_orientation_ = cloud_out.sensor_orientation_;
    }
    if (cloud_in.is_dense) {
        cloud_out = cloud_in;
    } else {
        size_t cloud_out_size = 0;
        size_t index = 0;
        std::for_each(std::execution::par, cloud_in.points.begin(), cloud_in.points.end(), [&](const auto& point) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                return;
            }
            cloud_out.points[cloud_out_size] = cloud_in.points[cloud_out_size];
            cloud_out_size++;
        });
        if (cloud_out_size != cloud_in.size()) {
            cloud_out.points.resize(cloud_out_size);
        }
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(cloud_out_size);
        cloud_out.is_dense = true;
    }
}
inline void SavePcd(const std::string& path, const PointCloudPtr& cloud) {
    cloud->height = 1;
    cloud->width = cloud->size();
    pcl::io::savePCDFileASCII(path, *cloud);
}
}  // namespace slam