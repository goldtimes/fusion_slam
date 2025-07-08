#include "map_node_build.hh"

namespace slam {
MapBuildNode::MapBuildNode(const ros::NodeHandle& nh) : nh_(nh) {
    LOG_INFO("MapBuildNode Initing.....");
    load_params();
    init_sub_pub();
    LOG_INFO("MapBuildNode initied.....");
}

void MapBuildNode::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
}
void MapBuildNode::standard_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
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