#include "map_node_build.hh"

namespace slam {
MapBuildNode::MapBuildNode(const ros::NodeHandle& nh) : nh_(nh) {
    LOG_INFO("MapBuildNode Initing.....");
    load_params();
    init_sub_pub();
    lidar_process_ptr_ = std::make_shared<LidarProcess>();
    LOG_INFO("MapBuildNode initied.....");
}

void MapBuildNode::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lck(mtx);
    double current_imu_time = imu_msg->header.stamp.toSec();
    if (current_imu_time < last_imu_time) {
        LOG_INFO("IMU detected loop, last_imu_time:{}, current_time:{}", last_imu_time, current_imu_time);
        imu_queque_.clear();
    }
    last_imu_time = current_imu_time;
    IMUData imu_data(imu_msg);
    imu_queque_.push_back(imu_data);
}
void MapBuildNode::standard_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
    std::lock_guard<std::mutex> lck(mtx);
    double current_lidar_time = lidar_msg->header.stamp.toSec();
    if (current_lidar_time < last_lidar_time) {
        LOG_INFO("lidar detected loop, last_imu_time:{}, current_time:{}", last_lidar_time, current_lidar_time);
        lidar_queque_.clear();
    }
    PointCloudPtr current_cloud(new PointCloud);
    lidar_process_ptr_->Process(lidar_msg, current_cloud);
    lidar_queque_.push_back(current_cloud);
    lidar_time_queue_.push_back(current_lidar_time);
}

void MapBuildNode::livox_callback(const livox_ros_driver2::CustomMsg::ConstPtr& lidar_msg) {
    std::lock_guard<std::mutex> lck(mtx);
    double current_lidar_time = lidar_msg->header.stamp.toSec();
    if (current_lidar_time < last_lidar_time) {
        LOG_INFO("lidar detected loop, last_imu_time:{}, current_time:{}", last_lidar_time, current_lidar_time);
        lidar_queque_.clear();
    }
    PointCloudPtr current_cloud(new PointCloud);
    lidar_process_ptr_->Process(lidar_msg, current_cloud);
    lidar_queque_.push_back(current_cloud);
    lidar_time_queue_.push_back(current_lidar_time);
}

void MapBuildNode::encoder_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
}
void MapBuildNode::gnss_callback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
}

void MapBuildNode::load_params() {
}
void MapBuildNode::init_sub_pub() {
}

void MapBuildNode::Run() {
}
}  // namespace slam