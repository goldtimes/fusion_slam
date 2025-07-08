/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-08 23:14:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-09 00:30:54
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/map_node_build.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <vector>
#include "common/common_lib.hh"
#include "common/logger.hh"
#include "lidar_process.hh"
#include "ros/rate.h"

namespace slam {
class MapBuildNode {
   public:
    struct MapBuildNodeConfig {
        std::string global_frame_;
        std::string local_frame_;
        std::string body_frame_;
        std::string imu_topic_;
        std::string lidar_topic_;
        std::string lidar_type_;
        double det_range;
        double cube_len;
        double map_resolution;
        double move_thresh;
        bool align_gravity;
        std::vector<double> imu_ext_rot;
        std::vector<double> imu_ext_pos;
    };

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

    bool SyncPackage(MeasureGroup& sync_package);

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
    std::shared_ptr<ros::Rate> local_rate_;
    std::shared_ptr<ros::Rate> loop_rate_;

    MeasureGroup sync_package;
    bool lidar_pushed = false;
    double lidar_mean_scantime = 0.0;
    int scan_num = 0;

    std::mutex mtx;
};
}  // namespace slam