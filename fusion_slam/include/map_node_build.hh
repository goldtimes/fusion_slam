#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "common/common_lib.hh"
#include "common/logger.hh"
#include "lidar_process.hh"

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
    void livox_callback(const livox_ros_driver2::CustomMsg::ConstPtr& lidar_msg);
    void encoder_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void gnss_callback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg);

    void load_params();
    void init_sub_pub();

   private:
    ros::NodeHandle nh_;
    MapBuildNodeConfig config_;
    LidarProcess::LidarProcessConfig lidar_process_config_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber encoder_sub_;

    std::deque<PointCloudPtr> lidar_queque_;
    std::deque<double> lidar_time_queue_;
    std::deque<IMUData> imu_queque_;
    std::deque<Encoder> encoder_queque_;
    std::deque<GNSS> gnss_queque_;
    //
    double last_imu_time = -1;
    double last_lidar_time = -1;
    double last_encoder_time = -1;
    double last_gnss_time = -1;

    std::shared_ptr<LidarProcess> lidar_process_ptr_;

    std::mutex mtx;
};
}  // namespace slam