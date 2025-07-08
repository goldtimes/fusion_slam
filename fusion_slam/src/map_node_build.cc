/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-08 23:14:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-08 23:54:13
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/map_node_build.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "map_node_build.hh"
#include "lidar_process.hh"
#include "ros/init.h"

namespace slam {
MapBuildNode::MapBuildNode(const ros::NodeHandle& nh) : nh_(nh) {
    LOG_INFO("MapBuildNode Initing.....");
    load_params();
    init_sub_pub();
    lidar_process_ptr_ = std::make_shared<LidarProcess>(lidar_process_config_);
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
    lidar_process_config_.N_SCAN = 6;
    lidar_process_config_.blind = 0.2;
    lidar_process_config_.point_filter_num = 2;
    nh_.param<std::string>("map_frame", config_.global_frame_, "map");
    nh_.param<std::string>("local_frame", config_.local_frame_, "local");
    nh_.param<std::string>("body_frame", config_.body_frame_, "body");
    nh_.param<std::string>("imu_topic", config_.imu_topic_, "/livox/imu");
    nh_.param<std::string>("livox_topic", config_.lidar_topic_, "/livox/lidar");
    std::string lidar_type;
    nh_.param<std::string>("lidar_type", lidar_type, "");
    if (lidar_type == "avia") {
        lidar_process_config_.lidar_type_ == LIDAR_TYPE::AVIA;
    } else if (lidar_type == "robosense") {
        lidar_process_config_.lidar_type_ == LIDAR_TYPE::ROBOSENSE;
    }

    double local_rate, loop_rate;
    nh_.param<double>("local_rate", local_rate, 20.0);
    nh_.param<double>("loop_rate", loop_rate, 1.0);
    nh_.param<int>("num_scan", lidar_process_config_.N_SCAN, 4);
    nh_.param<double>("blind", lidar_process_config_.blind, 0.2);
    nh_.param<int>("loop_rate", lidar_process_config_.point_filter_num, 2);
    local_rate_ = std::make_shared<ros::Rate>(local_rate);
    loop_rate_ = std::make_shared<ros::Rate>(loop_rate);
    nh_.param<double>("lio_builder/det_range", config_.det_range, 100.0);
    nh_.param<double>("lio_builder/cube_len", config_.cube_len, 500.0);
    nh_.param<double>("lio_builder/resolution", config_.map_resolution, 0.1);
    nh_.param<double>("lio_builder/move_thresh", config_.move_thresh, 1.5);
    nh_.param<bool>("lio_builder/align_gravity", config_.align_gravity, true);
    nh_.param<std::vector<double>>("lio_builder/imu_ext_rot", config_.imu_ext_rot, std::vector<double>());
    nh_.param<std::vector<double>>("lio_builder/imu_ext_pos", config_.imu_ext_pos, std::vector<double>());

    // nh_.param<bool>("loop_closure/activate", loop_closure_.mutableParams().activate, true);
    // nh_.param<double>("loop_closure/rad_thresh", loop_closure_.mutableParams().rad_thresh, 0.4);
    // nh_.param<double>("loop_closure/dist_thresh", loop_closure_.mutableParams().dist_thresh, 2.5);
    // nh_.param<double>("loop_closure/time_thresh", loop_closure_.mutableParams().time_thresh, 30.0);
    // nh_.param<double>("loop_closure/loop_pose_search_radius", loop_closure_.mutableParams().loop_pose_search_radius,
    //                   10.0);
    // nh_.param<int>("loop_closure/loop_pose_index_thresh", loop_closure_.mutableParams().loop_pose_index_thresh, 5);
    // nh_.param<double>("loop_closure/submap_resolution", loop_closure_.mutableParams().submap_resolution, 0.2);
    // nh_.param<int>("loop_closure/submap_search_num", loop_closure_.mutableParams().submap_search_num, 20);
    // nh_.param<double>("loop_closure/loop_icp_thresh", loop_closure_.mutableParams().loop_icp_thresh, 0.3);
}
void MapBuildNode::init_sub_pub() {
    imu_sub_ = nh_.subscribe(config_.imu_topic_, 200, &MapBuildNode::imu_callback, this);
    lidar_sub_ = lidar_process_config_.lidar_type_ == LIDAR_TYPE::AVIA
                     ? nh_.subscribe(config_.lidar_topic_, 10, &MapBuildNode::livox_callback, this)
                     : nh_.subscribe(config_.lidar_topic_, 10, &MapBuildNode::standard_pcl_callback, this);
}

void MapBuildNode::Run() {
    while (ros::ok()) {
        local_rate_->sleep();
        ros::spinOnce();
    }
}
}  // namespace slam