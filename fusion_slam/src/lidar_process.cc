#include "lidar_process.hh"
#include "common/lidar_model.hh"
#include "common/lidar_point_type.hh"
#include "common/logger.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cmath>
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

            if (cloud_ptr->points.back().time <= 0.0f){
                ComputePointOffsetTime(cloud_ptr, 10.0);
            }
            return cloud_ptr;
        }
        else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::OUSTER128){
            pcl::PointCloud<OusterPointXYZIRT> cloud_ouster;
            pcl::fromROSMsg(*lidar_msg, cloud_ouster);
            if (!lidar_msg->is_dense){
                RemoveNanFromPointCloud(cloud_ouster, cloud_ouster);
            }
            // 转换成自定义的xyzirt格式
            const size_t cloud_size = cloud_ouster.size();
            PointCloudPtr cloud_ptr(new PointCloud);
            cloud_ptr->resize(cloud_size);
            // 并行处理
            std::vector<size_t> indices(cloud_size);
            std::iota(indices.begin(), indices.end(), 0);
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](const size_t& index){
                PointXYZIRT point;
                point.x = cloud_ouster.points[index].x;
                point.y = cloud_ouster.points[index].y;
                point.z = cloud_ouster.points[index].z;
                point.intensity = cloud_ouster.points[index].intensity;
                point.ring = static_cast<uint8_t>(cloud_ouster.points[index].ring);
                // to us
                point.time = static_cast<float>(cloud_ouster.points[index].t * SystemConfig::GetInstance().lidar_config.lidar_time_scale);
                cloud_ptr->points[index] = point;
            });
            cloud_ptr->header = cloud_ouster.header;
            cloud_ptr->is_dense = true;

            if (cloud_ptr->points.back().time <= 0.0f){
                ComputePointOffsetTime(cloud_ptr, 10.0);
            }
            return cloud_ptr;

        }else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::ROBOSENSE16){
            pcl::PointCloud<RsPointXYZIRT> cloud_rs;
            pcl::fromROSMsg(*lidar_msg, cloud_rs);
            if (!lidar_msg->is_dense){
                RemoveNanFromPointCloud(cloud_rs, cloud_rs);
            }
            // 转换成自定义的xyzirt格式
            const size_t cloud_size = cloud_rs.size();
            PointCloudPtr cloud_ptr(new PointCloud);
            cloud_ptr->resize(cloud_size);
            // 并行处理
            std::vector<size_t> indices(cloud_size);
            std::iota(indices.begin(), indices.end(), 0);
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](const size_t& index){
                PointXYZIRT point;
                point.x = cloud_rs.points[index].x;
                point.y = cloud_rs.points[index].y;
                point.z = cloud_rs.points[index].z;
                point.intensity = cloud_rs.points[index].intensity;
                point.ring = static_cast<uint8_t>(cloud_rs.points[index].ring);
                // to us
                point.time = static_cast<float>((cloud_rs.points[index].timestamp - cloud_rs.points[0].timestamp) * SystemConfig::GetInstance().lidar_config.lidar_time_scale);
                cloud_ptr->points[index] = point;
            });
            cloud_ptr->header = cloud_rs.header;
            cloud_ptr->is_dense = true;

            if (cloud_ptr->points.back().time <= 0.0f){
                ComputePointOffsetTime(cloud_ptr, 10.0);
            }
            return cloud_ptr;
            
        }else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::LEISHEN16){
             pcl::PointCloud<LsPointXYZIRT> cloud_ls;
            pcl::fromROSMsg(*lidar_msg, cloud_ls);
            if (!lidar_msg->is_dense){
                RemoveNanFromPointCloud(cloud_ls, cloud_ls);
            }
            // 转换成自定义的xyzirt格式
            const size_t cloud_size = cloud_ls.size();
            PointCloudPtr cloud_ptr(new PointCloud);
            cloud_ptr->resize(cloud_size);
            // 并行处理
            std::vector<size_t> indices(cloud_size);
            std::iota(indices.begin(), indices.end(), 0);
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](const size_t& index){
                PointXYZIRT point;
                point.x = cloud_ls.points[index].x;
                point.y = cloud_ls.points[index].y;
                point.z = cloud_ls.points[index].z;
                point.intensity = cloud_ls.points[index].intensity;
                point.ring = static_cast<uint8_t>(cloud_ls.points[index].ring);
                // to cloud_ls
                point.time = static_cast<float>(cloud_ls.points[index].timestamp * SystemConfig::GetInstance().lidar_config.lidar_time_scale);
                cloud_ptr->points[index] = point;
            });
            cloud_ptr->header = cloud_ls.header;
            cloud_ptr->is_dense = true;

            if (cloud_ptr->points.back().time <= 0.0f){
                ComputePointOffsetTime(cloud_ptr, 10.0);
            }
            return cloud_ptr;
            
        }
        else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::MID360){
            pcl::PointCloud<LivoxMid360PointXYZITLT> cloud_mid360;
            pcl::fromROSMsg(*lidar_msg, cloud_mid360);
            if (!lidar_msg->is_dense){
                RemoveNanFromPointCloud(cloud_mid360, cloud_mid360);
            }
            // 转换成自定义的xyzirt格式
            const size_t cloud_size = cloud_mid360.size();
            PointCloudPtr cloud_ptr(new PointCloud);
            cloud_ptr->resize(cloud_size);
            // 并行处理
            std::vector<size_t> indices(cloud_size);
            std::iota(indices.begin(), indices.end(), 0);
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](const size_t& index){
                PointXYZIRT point;
                point.x = cloud_mid360.points[index].x;
                point.y = cloud_mid360.points[index].y;
                point.z = cloud_mid360.points[index].z;
                point.intensity = cloud_mid360.points[index].intensity;
                point.ring = 0;
                // to us
                point.time = static_cast<float>((cloud_mid360.points[index].timestamp - cloud_mid360.points[0].timestamp) * SystemConfig::GetInstance().lidar_config.lidar_time_scale);
                cloud_ptr->points[index] = point;
            });
            cloud_ptr->header = cloud_mid360.header;
            cloud_ptr->is_dense = true;

            if (cloud_ptr->points.back().time <= 0.0f){
                ComputePointOffsetTime(cloud_ptr, 10.0);
            }
            return cloud_ptr;
        }
        else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::AVIA){
        pcl::PointCloud<LivoxPointXYZITLT> cloud_livox;
        pcl::fromROSMsg(*lidar_msg, cloud_livox);

        const size_t point_size = cloud_livox.size();

        pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(new pcl::PointCloud<PointXYZIRT>());
        cloud_in_ptr->reserve(point_size);

        /// TODO: config
        uint8_t num_scans_ = 6;

        for (size_t i = 0; i < point_size; ++i) {
            if ((cloud_livox.points[i].line < num_scans_) &&
                ((cloud_livox.points[i].tag & 0x30) == 0x10 || (cloud_livox.points[i].tag & 0x30) == 0x00)) {
                PointXYZIRT point_xyzirt{};
                point_xyzirt.x = cloud_livox.points[i].x;
                point_xyzirt.y = cloud_livox.points[i].y;
                point_xyzirt.z = cloud_livox.points[i].z;
                point_xyzirt.intensity = cloud_livox.points[i].intensity;
                point_xyzirt.time = static_cast<float>(static_cast<double>(cloud_livox.points[i].time) *
                    SystemConfig::GetInstance().lidar_config.lidar_time_scale);
                cloud_in_ptr->emplace_back(point_xyzirt);
            }
        }

        cloud_in_ptr->header = cloud_livox.header;
        cloud_in_ptr->is_dense = true;

        return cloud_in_ptr;
            
        }else if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::NONE){
            
        }else{
            LOG_ERROR("unknow lidar type");
        }
    }

    // 标记每条线上的第一个点，然后根据角度差/角速度得到时间信息
    // 这里没有标记cloud为const
    void LidarProcess::ComputePointOffsetTime(const PointCloudPtr& cloud, const double lidar_rate){
        const int lidar_scan_num = LidarModel::GetInstance()->vertical_scan_num_;
        const auto& cloud_size = cloud->size();
        const double lidar_omega = 2 * M_PI * lidar_rate;

        // 标记信息
        std::vector<bool> yaw_first_scan(lidar_scan_num, 0.0);
        std::vector<bool> is_first(lidar_scan_num, true);
        std::vector<float> time_last(lidar_scan_num, 0.0);
        for (size_t i = 0; i < cloud_size; ++i){
            const int ring = cloud->points[i].ring;
            if (ring >= lidar_scan_num){
                LOG_INFO("lidar cloud point ring invalid");
                continue;
            }
            // 计算这个点的角度
            const double yaw = std::atan2(cloud->points[i].y, cloud->points[i].x);
            if (is_first[ring]){
                is_first[ring] = false;
                yaw_first_scan[ring] = yaw;
                time_last[ring] = 0.0f;
                continue;
            }

            if (yaw <= yaw_first_scan[ring]){
                cloud->points[i].time = static_cast<float>((yaw_first_scan[ring] - yaw) / lidar_omega );
            }else{
                cloud->points[i].time = static_cast<float>(((yaw_first_scan[ring] - yaw) + 2 * M_PI) / lidar_omega );
            }

            if (cloud->points[i].time < time_last[ring]){   
                 cloud->points[i].time += static_cast<float>(2.0 * M_PI / lidar_omega);
            }
            time_last[ring] = cloud->points[i].time;
        }
    }


}