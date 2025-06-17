#pragma once 

#include "pcl/point_cloud.h"
#include "sensors/lidar.hh"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <execution>

namespace slam {
    template <typename PointT>
    inline void RemoveNanFromPointCloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out){
        if (&cloud_in != cloud_out){
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
            cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
            cloud_out.sensor_orientation_ = cloud_out.sensor_orientation_;
        }
        if (cloud_in.is_dense){
            cloud_out = cloud_in;
        }
        else {
            size_t cloud_out_size = 0;
            size_t index = 0;
            std::for_each(std::execution::par, cloud_in.points.begin(), cloud_in.points.end(), [&](const auto& point){
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)){
                    return;
                }
                cloud_out.points[cloud_out_size] = cloud_in.points[cloud_out_size];
                cloud_out_size++;
            });
            if (cloud_out_size != cloud_in.size()){
                cloud_out.points.resize(cloud_out_size);
            }
            cloud_out.height = 1;
            cloud_out.width = static_cast<uint32_t>(cloud_out_size);
            cloud_out.is_dense = true;
        }

    }
}