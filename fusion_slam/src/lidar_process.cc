#include "lidar_process.hh"
#include "common/logger.hh"
#include "sensors/lidar.hh"

namespace slam {
LidarProcess::LidarProcess(LidarProcessConfig config) : config_(config) {
    LOG_INFO("LidarProcess Inited!");
    full_cloud_.reset(new PointCloud);
}

void LidarProcess::Process(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg, PointCloudPtr& pcl_out) {
    avia_handler(livox_msg);
    pcl_out = full_cloud_;
}
void LidarProcess::Process(const sensor_msgs::PointCloud2::ConstPtr& msg, PointCloudPtr& pcl_out) {
    if (config_.lidar_type_ == LIDAR_TYPE::ROBOSENSE) {
        robosen_handler(msg);
    } else if (config_.lidar_type_ == LIDAR_TYPE::MID360) {
        mid360_handler(msg);
    } else if (config_.lidar_type_ == LIDAR_TYPE::VELODYNE16) {
        velo_handler(msg);
    } else if (config_.lidar_type_ == LIDAR_TYPE::OUSTER64) {
        ouster_handler(msg);
    } else {
        LOG_ERROR("Unkown Lidar Type");
    }
    pcl_out = full_cloud_;
}
void LidarProcess::avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
    size_t num_points = msg->point_num;
    LOG_INFO("avia_handler get num points:{}", num_points);
    int valid_num = 0;
    full_cloud_->clear();
    full_cloud_->reserve(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        if ((msg->points[i].line < config_.N_SCAN) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            valid_num++;
            if (valid_num % config_.point_filter_num != 0) {
                continue;
            }
            PointType point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            point.intensity = msg->points[i].reflectivity;
            // ns -> ms
            point.curvature = msg->points[i].offset_time / float(1e6);
            double dist = point.x * point.x + point.y * point.y + point.z * point.z;
            if (dist > config_.blind) {
                full_cloud_->push_back(point);
            }
        }
    }
    LOG_INFO("avia_handler after filter get num points:{}", full_cloud_->points.size());
}
void LidarProcess::robosen_handler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}
void LidarProcess::velo_handler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}
void LidarProcess::mid360_handler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}
void LidarProcess::ouster_handler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}

}  // namespace slam