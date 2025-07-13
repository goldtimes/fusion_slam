/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-08 23:14:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-14 00:10:46
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/map_node_build.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "map_node_build.hh"
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include "common/common_lib.hh"
#include "fastlio_odom/fastlio_odom.hh"
#include "lidar_process.hh"
#include "loop_closure.hh"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "sensor_msgs/PointCloud2.h"

namespace slam {
MapBuildNode::MapBuildNode(const ros::NodeHandle& nh) : nh_(nh) {
    LOG_INFO("MapBuildNode Initing.....");
    load_params();
    init_sub_pub();
    lidar_process_ptr_ = std::make_shared<LidarProcess>(lidar_process_config_);
    fastlio_odom_ptr_ = std::make_shared<FastlioOdom>(fastlio_odom_config_);

    loop_closure_.setRate(loop_rate_);
    shared_data = std::make_shared<SharedData>();
    loop_closure_.setShared(shared_data);
    loop_closure_.Init();
    // 创建线程
    loop_closure_thread_ = std::make_shared<std::thread>(std::ref(loop_closure_));
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
    nh_.param<std::string>("global_frame", config_.global_frame_, "map");
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
    nh_.param<bool>("loop_closure/activate", loop_closure_.mutableParams().active, true);
    nh_.param<double>("loop_closure/rad_thresh", loop_closure_.mutableParams().rad_thresh, 0.4);
    nh_.param<double>("loop_closure/dist_thresh", loop_closure_.mutableParams().dist_thresh, 2.5);
    nh_.param<double>("loop_closure/time_thresh", loop_closure_.mutableParams().time_thresh, 30.0);
    nh_.param<double>("loop_closure/loop_pose_search_radius", loop_closure_.mutableParams().loop_pose_search_thresh,
                      10.0);
    nh_.param<int>("loop_closure/loop_pose_index_thresh", loop_closure_.mutableParams().loop_pose_index_thresh, 5);
    nh_.param<double>("loop_closure/submap_resolution", loop_closure_.mutableParams().submap_resolution, 0.2);
    nh_.param<int>("loop_closure/submap_search_num", loop_closure_.mutableParams().submap_search_num, 20);
    nh_.param<double>("loop_closure/loop_icp_thresh", loop_closure_.mutableParams().loop_icp_thresh, 0.3);

    fastlio_odom_config_.move_thresh = config_.move_thresh;
    fastlio_odom_config_.align_gravity = config_.align_gravity;
    fastlio_odom_config_.cube_len = config_.cube_len;
    fastlio_odom_config_.resolution = config_.map_resolution;
    fastlio_odom_config_.imu_ext_rot = config_.imu_ext_rot;
    fastlio_odom_config_.imu_ext_pos = config_.imu_ext_pos;
}
void MapBuildNode::init_sub_pub() {
    imu_sub_ = nh_.subscribe(config_.imu_topic_, 200, &MapBuildNode::imu_callback, this);
    lidar_sub_ = lidar_process_config_.lidar_type_ == LIDAR_TYPE::AVIA
                     ? nh_.subscribe(config_.lidar_topic_, 10, &MapBuildNode::livox_callback, this)
                     : nh_.subscribe(config_.lidar_topic_, 10, &MapBuildNode::standard_pcl_callback, this);
    body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lio_node/body_cloud", 10);
    world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lio_node/world_cloud", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lio_node/odom", 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 10);
    // loop_mark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/loop_mark", 10);
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
        // 发布tf
        // odom->map的变换一开始是重合的，但是里程计因为有漂移之后，我们检测到了回环，优化了整个里程计的位姿之后，这个offset_rot,offset_trans就代表了odom->map的偏移，肯定不重合了
        tf_broadcaster_.sendTransform(eigen2Transform(shared_data->offset_rot, shared_data->offset_trans,
                                                      config_.global_frame_, config_.local_frame_, current_time_));
        auto T_body_to_local = eigen2Transform(current_state_.rot.matrix(), current_state_.pos, config_.local_frame_,
                                               config_.body_frame_, sync_package.lidar_end_time);
        tf_broadcaster_.sendTransform(T_body_to_local);

        publishOdom(eigen2Odometry(current_state_.rot.toRotationMatrix(), current_state_.pos, config_.local_frame_,
                                   config_.body_frame_, current_time_));

        publishCloud(body_cloud_pub_,
                     pcl2msg(fastlio_odom_ptr_->cloudUndistortedBody(), config_.body_frame_, current_time_));
        publishCloud(world_cloud_pub_,
                     pcl2msg(fastlio_odom_ptr_->GetcCloudWorld(), config_.local_frame_, current_time_));
        // 接下来需要处理关键位姿，将关键位姿发布给闭环检测线程
        addKeypose();
        publishLocalPath();
        publishGlobalPath();
        publishLoopMark();
    }
    loop_closure_thread_->join();
    std::cout << "MAPPING NODE IS DOWN!" << std::endl;
}

void MapBuildNode::addKeypose() {
    int idx = shared_data->key_poses.size();
    if (shared_data->key_poses.empty()) {
        std::lock_guard<std::mutex> lck(shared_data->shared_data_mutex);
        shared_data->key_poses.emplace_back(idx, current_time_, current_state_.rot.toRotationMatrix(),
                                            current_state_.pos);
        shared_data->key_poses.back().addOffset(shared_data->offset_rot, shared_data->offset_trans);
        shared_data->key_pose_added = true;
        // 存储世界坐标系下的点云
        shared_data->cloud_historys.push_back(fastlio_odom_ptr_->cloudUndistortedBody());
        return;
    }
    // 关键帧的判断
    Pose6D& last_pose = shared_data->key_poses.back();
    M3D delta_rot = last_pose.local_rot.transpose() * current_state_.rot.toRotationMatrix();
    V3D delta_trans = last_pose.local_rot.transpose() * (current_state_.pos - last_pose.local_pos);
    V3D rpy = rotate2rpy(delta_rot);
    if (delta_trans.norm() > loop_closure_.mutableParams().dist_thresh ||
        std::abs(rpy(0)) > loop_closure_.mutableParams().rad_thresh ||
        std::abs(rpy(1)) > loop_closure_.mutableParams().rad_thresh ||
        std::abs(rpy(2)) > loop_closure_.mutableParams().rad_thresh) {
        std::lock_guard<std::mutex> lock(shared_data->shared_data_mutex);
        shared_data->key_poses.emplace_back(idx, current_time_, current_state_.rot.toRotationMatrix(),
                                            current_state_.pos);
        // 未检测到回环的时候这里offset都为0，所以global_path和local_path几乎是重合的
        shared_data->key_poses.back().addOffset(shared_data->offset_rot, shared_data->offset_trans);
        // shared_data_->key_poses.back().gravity = current_state_.get_g();
        shared_data->key_pose_added = true;
        shared_data->cloud_historys.push_back(fastlio_odom_ptr_->cloudUndistortedBody());
    }
    LOG_INFO("key pose size: {}", shared_data->key_poses.size());
}

void MapBuildNode::publishLocalPath() {
    if (local_path_pub_.getNumSubscribers() == 0) return;

    if (shared_data->key_poses.empty()) return;

    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time().fromSec(current_time_);
    for (Pose6D& p : shared_data->key_poses) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time().fromSec(current_time_);
        pose.pose.position.x = p.local_pos(0);
        pose.pose.position.y = p.local_pos(1);
        pose.pose.position.z = p.local_pos(2);
        Eigen::Quaterniond q(p.local_rot);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path.poses.push_back(pose);
    }
    local_path_pub_.publish(path);
}

void MapBuildNode::publishGlobalPath() {
    if (global_path_pub_.getNumSubscribers() == 0) return;

    if (shared_data->key_poses.empty()) return;
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time().fromSec(current_time_);
    for (Pose6D& p : shared_data->key_poses) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time().fromSec(current_time_);
        pose.pose.position.x = p.global_pos(0);
        pose.pose.position.y = p.global_pos(1);
        pose.pose.position.z = p.global_pos(2);
        Eigen::Quaterniond q(p.global_rot);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path.poses.push_back(pose);
    }
    global_path_pub_.publish(path);
}
void MapBuildNode::publishLoopMark() {
    if (loop_mark_pub_.getNumSubscribers() == 0) return;
    if (shared_data->loop_history.empty()) return;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker nodes_marker;

    nodes_marker.header.frame_id = "map";
    nodes_marker.header.stamp = ros::Time().fromSec(current_time_);
    nodes_marker.ns = "loop_nodes";
    nodes_marker.id = 0;
    nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes_marker.action = visualization_msgs::Marker::ADD;
    nodes_marker.pose.orientation.w = 1.0;
    nodes_marker.scale.x = 0.3;
    nodes_marker.scale.y = 0.3;
    nodes_marker.scale.z = 0.3;
    nodes_marker.color.r = 1.0;
    nodes_marker.color.g = 0.8;
    nodes_marker.color.b = 0.0;
    nodes_marker.color.a = 1.0;

    visualization_msgs::Marker edges_marker;
    edges_marker.header.frame_id = "map";
    edges_marker.header.stamp = ros::Time().fromSec(current_time_);
    edges_marker.ns = "loop_edges";
    edges_marker.id = 1;
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::Marker::ADD;
    edges_marker.pose.orientation.w = 1.0;
    edges_marker.scale.x = 0.1;

    edges_marker.color.r = 0.0;
    edges_marker.color.g = 0.8;
    edges_marker.color.b = 0.0;
    edges_marker.color.a = 1.0;
    for (auto& p : shared_data->loop_history) {
        Pose6D& p1 = shared_data->key_poses[p.first];
        Pose6D& p2 = shared_data->key_poses[p.second];
        geometry_msgs::Point point1;
        point1.x = p1.global_pos(0);
        point1.y = p1.global_pos(1);
        point1.z = p1.global_pos(2);
        geometry_msgs::Point point2;
        point2.x = p2.global_pos(0);
        point2.y = p2.global_pos(1);
        point2.z = p2.global_pos(2);
        nodes_marker.points.push_back(point1);
        nodes_marker.points.push_back(point2);
        edges_marker.points.push_back(point1);
        edges_marker.points.push_back(point2);
    }
    marker_array.markers.push_back(nodes_marker);
    marker_array.markers.push_back(edges_marker);
    loop_mark_pub_.publish(marker_array);
}
}  // namespace slam