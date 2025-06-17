#include "lidar_process.hh"
#include "common/lidar_model.hh"
#include "common/lidar_point_type.hh"
#include "common/logger.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <execution>
#include <numeric>
#include <vector>
#include "common/pointcloud_utils.hh"
#include "sensors/lidar.hh"

namespace slam {
    LidarProcess::LidarProcess(){
        
    }

    PointCloudPtr LidarProcess::ConvertMessageToCloud(sensor_msgs::PointCloud2::ConstPtr& lidar_msg){
        if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::Velodyne){
            pcl::PointCloud<VelodynePointXYZIRT> cloud_vel;
            pcl::fromROSMsg(*lidar_msg, cloud_vel);
            if (!lidar_msg->is_dense){
                RemoveNanFromPointCloud(cloud_vel, cloud_vel);
            }
            // 转换成自定义的xyzirt格式
            const size_t cloud_size = cloud_vel.size();
            PointCloudPtr cloud_ptr(new PointCloud);
            cloud_ptr->resize(cloud_size);
            // 并行处理
            std::vector<size_t> indices(cloud_size);
            std::iota(indices.begin(), indices.end(), 0);
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](const size_t& index){
                PointXYZIRT point;
                point.x = cloud_vel.points[index].x;
                point.y = cloud_vel.points[index].y;
                point.z = cloud_vel.points[index].z;
                point.intensity = cloud_vel.points[index].intensity;
                point.ring = static_cast<uint8_t>(cloud_vel.points[index].ring);
                // to us
                point.time = static_cast<float>(cloud_vel.points[index].time * SystemConfig::GetInstance().lidar_config.lidar_time_scale);
                cloud_ptr->points[index] = point;
            });
            cloud_ptr->header = cloud_vel.header;
            cloud_ptr->is_dense = true;

        }
        else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::OUSTER128){

        }else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::ROBOSENSE16){
            
        }else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::LEISHEN16){
            
        }
        else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::MID360){
            
        }
        else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::AVIA){
            
        }else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::NONE){
            
        }else{
            LOG_ERROR("unknow lidar type");
        }
    }

}