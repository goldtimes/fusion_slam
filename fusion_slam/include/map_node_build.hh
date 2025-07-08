#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "common/common_lib.hh"
#include "common/logger.hh"

namespace slam {
class MapBuildNode {
   public:
    struct MapBuildNodeConfig {};

   public:
    MapBuildNode(const ros::NodeHandle& nh);
    void Run();

   private:
    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void standard_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);
    void encoder_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void gnss_callback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg);

    void load_params();
    void init_sub_pub();

   private:
    ros::NodeHandle nh_;
    MapBuildNodeConfig config_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber encoder_sub_;
};
}  // namespace slam