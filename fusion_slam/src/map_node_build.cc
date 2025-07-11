/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-08 23:14:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-12 00:52:32
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/map_node_build.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "map_node_build.hh"
#include "common/common_lib.hh"
#include "fastlio_odom/fastlio_odom.hh"
#include "lidar_process.hh"
#include "ros/init.h"
#include "sensor_msgs/PointCloud2.h"

namespace slam {
MapBuildNode::MapBuildNode(const ros::NodeHandle& nh) : nh_(nh) {
    LOG_INFO("MapBuildNode Initing.....");
    load_params();
    init_sub_pub();
    lidar_process_ptr_ = std::make_shared<LidarProcess>(lidar_process_config_);
    fastlio_odom_ptr_ = std::make_shared<FastlioOdom>(fastlio_odom_config_);
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
    std::vector<double> imu_ext_rot, imu_ext_pose;
    nh_.param<std::vector<double>>("lio_builder/imu_ext_rot", imu_ext_rot, std::vector<double>());
    nh_.param<std::vector<double>>("lio_builder/imu_ext_pos", imu_ext_pose, std::vector<double>());
    config_.imu_ext_rot = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(imu_ext_rot.data());
    config_.imu_ext_pos << imu_ext_pose[0], imu_ext_pose[1], imu_ext_pose[2];
    std::cout << "imu_ext_rot: \n" << config_.imu_ext_rot << std::endl;
    std::cout << "imu_ext_pos: \n" << config_.imu_ext_pos.transpose() << std::endl;
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

    fastlio_odom_config_.move_thresh = config_.move_thresh;
    fastlio_odom_config_.align_gravity = config_.align_gravity;
    fastlio_odom_config_.cube_len = config_.cube_len;
    fastlio_odom_config_.resolution = config_.map_resolution;
    fastlio_odom_config_.imu_ext_rot = config_.imu_ext_rot;
    fastlio_odom_config_.imu_ext_pos = config_.imu_ext_pos;
    // fastlio_odom_config_.esikf_max_iteration = config_.
}
void MapBuildNode::init_sub_pub() {
    imu_sub_ = nh_.subscribe(config_.imu_topic_, 200, &MapBuildNode::imu_callback, this);
    lidar_sub_ = lidar_process_config_.lidar_type_ == LIDAR_TYPE::AVIA
                     ? nh_.subscribe(config_.lidar_topic_, 10, &MapBuildNode::livox_callback, this)
                     : nh_.subscribe(config_.lidar_topic_, 10, &MapBuildNode::standard_pcl_callback, this);
    body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lio_node/body_cloud", 10);
    world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lio_node/world_cloud", 10);
}

bool MapBuildNode::SyncPackage(MeasureGroup& sync_package) {
    if (imu_queque_.empty() || lidar_queque_.empty() && lidar_time_queue_.empty()) {
        return false;
    }
    if (!lidar_pushed) {
        sync_package.current_lidar = lidar_queque_.front();
        sync_package.lidar_begin_time = lidar_time_queue_.front();
        // 点云有问题
        if (sync_package.current_lidar->points.size() <= 1) {
            sync_package.lidar_end_time = sync_package.lidar_end_time + lidar_mean_scantime;
            LOG_ERROR("Too few input point cloud!");
        }
        // 最尾部的时间 < 一半的scan_time
        // sync_package.current_lidar->points.back().curvature / double(1000) 这里是相对于第一个点的偏移时间
        else if (sync_package.current_lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
            sync_package.lidar_end_time = sync_package.lidar_begin_time + lidar_mean_scantime;
        } else {
            scan_num++;
            sync_package.lidar_end_time =
                sync_package.lidar_begin_time + sync_package.current_lidar->points.back().curvature / double(1000);
            lidar_mean_scantime +=
                (sync_package.current_lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        lidar_pushed = true;
    }
    // LOG_INFO("lidar_begin_time:{},lidar_end_time:{}, lidar_mean_scantime:{}", sync_package.lidar_begin_time,
    //          sync_package.lidar_end_time, lidar_mean_scantime);
    // 处理imu
    if (last_imu_time < sync_package.lidar_end_time) {
        return false;
    }
    double imu_time = imu_queque_.front().timestamped_;
    sync_package.imu_queue_.clear();
    while (!imu_queque_.empty() && (imu_time < sync_package.lidar_end_time)) {
        imu_time = imu_queque_.front().timestamped_;
        if (imu_time > sync_package.lidar_end_time) {
            break;
        }
        sync_package.imu_queue_.push_back(imu_queque_.front());
        imu_queque_.pop_front();
    }
    // LOG_INFO("find imu size:{}", sync_package.imu_queue_.size());
    // if (!sync_package.imu_queue_.empty()) {
    // LOG_INFO("find imu begin time:{}, end_time:{}", sync_package.imu_queue_.front().timestamped_,
    //          sync_package.imu_queue_.back().timestamped_);
    // }
    lidar_time_queue_.pop_front();
    lidar_queque_.pop_front();
    lidar_pushed = false;
    return true;
}

void MapBuildNode::Run() {
    while (ros::ok()) {
        local_rate_->sleep();
        ros::spinOnce();
        MeasureGroup sync_package;
        if (!SyncPackage(sync_package)) {
            continue;
        }
        // LOG_INFO("SyncPackage Success");
        fastlio_odom_ptr_->mapping(sync_package);
        if (fastlio_odom_ptr_->GetSystemStatus() == SYSTEM_STATUES::INITIALIZE) {
            continue;
        }
        current_time_ = sync_package.lidar_end_time;
        current_state_ = fastlio_odom_ptr_->GetCurrentState();
        auto T_body_to_local = eigen2Transform(current_state_.rot.matrix(), current_state_.pos, config_.local_frame_,
                                               config_.body_frame_, sync_package.lidar_end_time);
        tf_broadcaster_.sendTransform(T_body_to_local);

        publishOdom(eigen2Odometry(current_state_.rot.toRotationMatrix(), current_state_.pos, config_.local_frame_,
                                   config_.body_frame_, current_time_));

        publishCloud(body_cloud_pub_,
                     pcl2msg(fastlio_odom_ptr_->cloudUndistortedBody(), config_.body_frame_, current_time_));
        publishCloud(world_cloud_pub_,
                     pcl2msg(fastlio_odom_ptr_->GetcCloudWorld(), config_.local_frame_, current_time_));

        // publishLocalPath();
    }
}
}  // namespace slam