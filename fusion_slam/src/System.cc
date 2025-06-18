#include "System.hh"
// #include <livox_ros_driver/CustomMsg.h>
#include <memory>
#include <string>
#include "common/PoseTrans.hpp"
#include "common/lidar_model.hh"
#include "common/logger.hpp"
#include "ros/init.h"
#include "ros/rate.h"
#include "sensors/imu.hh"

namespace slam {
System::System(const ros::NodeHandle& nh) : nh_(nh) {
    // 初始化配置文件
    InitConfigAndPrint();
    InitLidarModel();

    // ros接口
    LOG_INFO("init sub pub");
    InitSubPub();
    lidar_process_ = std::make_shared<LidarProcess>();
}
void System::InitConfigAndPrint() {
    SystemConfig& config = SystemConfig::GetInstance();

    // sensor topic name
    nh_.getParam("sensor_topic/lidar_topic", config.lidar_topic);
    nh_.getParam("sensor_topic/imu_topic", config.imu_topic);
    LOG_INFO("lidar_topic:{}", config.lidar_topic);
    LOG_INFO("imu_topic:{}", config.imu_topic);
    int temp;
    // nh_.param("slam_mode", temp, 0);
    // slam_mode_ = static_cast<SLAM_MODE>(temp);

    // lidar config parameters
    nh_.param<std::string>("lidar/lidar_sensor_type", config.lidar_config.lidar_type_, "");
    nh_.param("lidar/lidar_point_jump_span", config.lidar_config.lidar_point_filter, 2);
    nh_.param("lidar/lidar_scan", config.lidar_config.lidar_scan, 16);
    nh_.param("lidar/lidar_lower_angle", config.lidar_config.lidar_lower_angle, 0.0);
    nh_.param("lidar/lidar_horizon_scan", config.lidar_config.lidar_horizon_scan, 6);
    nh_.param("lidar/lidar_vertical_resolution", config.lidar_config.lidar_vertical_resolution, 1.6);
    nh_.param("lidar/lidar_use_min_distance", config.lidar_config.lidar_min_dist, 0.15);
    nh_.param("lidar/lidar_use_max_distance", config.lidar_config.lidar_max_dist, 50.0);
    nh_.param("lidar/lidar_point_time_scale", config.lidar_config.lidar_time_scale, 1e9);
    nh_.param("lidar/lidar_rotation_noise_std", config.lidar_config.lidar_rotation_noise, 0.01);
    nh_.param("lidar/lidar_position_noise_std", config.lidar_config.lidar_position_noise, 0.03);
    LOG_INFO("lidar_sensor_type:{}", config.lidar_config.lidar_type_);
    LOG_INFO("lidar_point_jump_span:{}", config.lidar_config.lidar_point_filter);
    LOG_INFO("lidar_scan:{}", config.lidar_config.lidar_scan);
    LOG_INFO("lidar_lower_angle:{}", config.lidar_config.lidar_lower_angle);
    LOG_INFO("lidar_horizon_scan:{}", config.lidar_config.lidar_horizon_scan);
    LOG_INFO("lidar_vertical_resolution:{}", config.lidar_config.lidar_vertical_resolution);
    LOG_INFO("lidar_use_min_distance:{}", config.lidar_config.lidar_min_dist);
    LOG_INFO("lidar_use_max_distance:{}", config.lidar_config.lidar_max_dist);
    LOG_INFO("lidar_time_scale:{}", config.lidar_config.lidar_time_scale);
    LOG_INFO("lidar_rotation_noise:{}", config.lidar_config.lidar_rotation_noise);
    LOG_INFO("lidar_position_noise_std:{}", config.lidar_config.lidar_position_noise);

    // imu config parameters
    // nh_.param("imu/has_orientation", config.imu_has_orientation_, false);
    // nh_.param("imu/init_acc_bias", config.imu_init_acc_bias_, DoubleNaN);
    // nh_.param("imu/init_gyro_bias", config.imu_init_gyro_bias_, DoubleNaN);
    // nh_.param("imu/acc_noise_std", config.imu_acc_noise_std_, DoubleNaN);
    // nh_.param("imu/gyro_noise_std", config.imu_gyro_noise_std_, DoubleNaN);
    // nh_.param("imu/acc_rw_noise_std", config.imu_acc_rw_noise_std_, DoubleNaN);
    // nh_.param("imu/gyro_rw_noise_std", config.imu_gyro_rw_noise_std_, DoubleNaN);
    // nh_.param("imu/data_searcher_buffer_size", config.imu_data_searcher_buffer_size_, IntNaN);

    // gravity
    nh_.param("gravity", config.imu_init_config.gravity_norm_, 9.81);
    LOG_INFO("lidar_position_noise_std:{}", config.lidar_config.lidar_position_noise);

    // calibration parameters
    std::vector<double> lidar_to_imu;
    nh_.getParam("calibration/lidar_to_imu", lidar_to_imu);
    Eigen::Matrix4d T_I_L = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(lidar_to_imu.data());
    config.T_I_L = PoseTranseD(T_I_L.block<3, 3>(0, 0), T_I_L.block<3, 1>(0, 3));
    // LOG_INFO("T_I_L:\n{}", config.T_I_L);
    // frontend config parameters
    // nh_.param("frontend/registration_and_searcher_mode", config.registration_and_searcher_mode_, StringEmpty);
    // nh_.param("frontend/feature/corner_thres", config.loam_feature_corner_thres_, FloatNaN);
    // nh_.param("frontend/feature/planar_thres", config.loam_feature_planar_thres_, FloatNaN);
    // nh_.param("frontend/feature/planar_voxel_filter_size", config.loam_feature_planar_voxel_filter_size_, FloatNaN);
    // nh_.param("frontend/feature/corner_voxel_filter_size", config.loam_feature_corner_voxel_filter_size_, FloatNaN);
    // nh_.param("frontend/registration/line_ratio_thres", config.registration_line_ratio_thres_, FloatNaN);
    // nh_.param("frontend/registration/point_search_thres", config.registration_point_search_thres_, FloatNaN);
    // nh_.param("frontend/registration/point_to_planar_thres", config.registration_point_to_planar_thres_, FloatNaN);
    // nh_.param("frontend/registration/local_planar_map_size", config.registration_local_planar_map_size_, IntNaN);
    // nh_.param("frontend/registration/local_corner_map_size", config.registration_local_corner_map_size_, IntNaN);
    // nh_.param("frontend/registration/keyframe_delta_distance", config.registration_keyframe_delta_dist_, DoubleNaN);
    // nh_.param("frontend/registration/keyframe_delta_rotation", config.registration_keyframe_delta_rotation_,
    // DoubleNaN); nh_.param("frontend/registration/rotation_converge_thres",
    // config.registration_rotation_converge_thres_, FloatNaN);
    // nh_.param("frontend/registration/position_converge_thres", config.registration_position_converge_thres_,
    // FloatNaN); nh_.param("frontend/registration/local_corner_voxel_filter_size",
    // config.registration_local_corner_filter_size_,
    //           FloatNaN);
    // nh_.param("frontend/registration/local_planar_voxel_filter_size", config.registration_local_planar_filter_size_,
    //           FloatNaN);
    // nh_.param("frontend/registration/local_map_size", config.registration_local_map_size_, IntNaN);
    // nh_.param("frontend/registration/local_map_cloud_filter_size", config.registration_local_map_cloud_filter_size_,
    //           FloatNaN);
    // nh_.param("frontend/registration/source_cloud_filter_size", config.registration_source_cloud_filter_size_,
    //           FloatNaN);
    // nh_.param("frontend/registration/optimization_iter_num", config.registration_opti_iter_num_, IntNaN);
    // nh_.param("frontend/registration/ndt_voxel_size", config.registration_ndt_voxel_size_, DoubleNaN);
    // nh_.param("frontend/registration/ndt_outlier_threshold", config.registration_ndt_outlier_threshold_, DoubleNaN);
    // nh_.param("frontend/registration/ndt_min_points_in_voxel", config.registration_ndt_min_points_in_voxel_, IntNaN);
    // nh_.param("frontend/registration/ndt_max_points_in_voxel", config.registration_ndt_max_points_in_voxel_, IntNaN);
    // nh_.param("frontend/registration/ndt_min_effective_pts", config.registration_ndt_min_effective_pts_, IntNaN);
    // nh_.param("frontend/registration/ndt_capacity", config.registration_ndt_capacity_, IntNaN);
    // nh_.param("frontend/fusion_method", config.fusion_method_, StringEmpty);
    // nh_.param("frontend/fusion_opti_iters", config.frontend_fusion_opti_iters_, IntNaN);

    // // system config parameters
    // nh_.param("system/keyframe_delta_distance", config.system_keyframe_delta_dist_, DoubleNaN);
    // nh_.param("system/keyframe_delta_rotation", config.system_keyframe_delta_rotation_, DoubleNaN);
    // nh_.param("system/enable_loopclosure", config.system_enable_loopclosure_, false);
    // nh_.param("system/enable_visualize_global_map", config.system_enable_visualize_global_map_, false);
    // nh_.param("system/global_map_visualization_resolution", config.system_global_map_visualization_resolution_,
    //           FloatNaN);
    // // split map
    // nh_.param("system/tile_map_grid_size", config.tile_map_grid_size_, DoubleNaN);

    // // loopclosure config parameters
    // nh_.param("loopclosure/skip_near_loopclosure_threshold", config.lc_skip_near_loopclosure_threshold_, IntNaN);
    // nh_.param("loopclosure/skip_near_keyframe_threshold", config.lc_skip_near_keyframe_threshold_, IntNaN);
    // nh_.param("loopclosure/candidate_local_map_left_range", config.lc_candidate_local_map_left_range_, IntNaN);
    // nh_.param("loopclosure/candidate_local_map_right_range", config.lc_candidate_local_map_right_range_, IntNaN);
    // nh_.param("loopclosure/loopclosure_local_map_left_range", config.lc_loopclosure_local_map_left_range_, IntNaN);
    // nh_.param("loopclosure/near_neighbor_distance_threshold", config.lc_near_neighbor_distance_threshold_,
    // DoubleNaN); nh_.param("loopclosure/registration_converge_threshold", config.lc_registration_converge_threshold_,
    // FloatNaN);
}

void System::InitSubPub() {
    const auto& config = SystemConfig::GetInstance();
    if (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::AVIA) {
        // lidar_sub_ = nh_.subscribe(config.lidar_topic, 10, &System::LivoxLidarCallback, this);
    } else {
        lidar_sub_ = nh_.subscribe(config.lidar_topic, 10, &System::LidarCallback, this);
    }
    imu_sub_ = nh_.subscribe(config.imu_topic, 200, &System::ImuCallback, this);
}

void System::InitLidarModel() {
    LOG_INFO("Init Lidar Model");
    if (SystemConfig::GetInstance().lidar_config.lidar_type_ == "None") {
        LidarModel::GetInstance(SystemConfig::GetInstance().lidar_config.lidar_type_);
        int lidar_horizon_scan = SystemConfig::GetInstance().lidar_config.lidar_horizon_scan;
        LidarModel::GetInstance()->horizon_scan_num_ = lidar_horizon_scan;
        LidarModel::GetInstance()->vertical_scan_num_ = SystemConfig::GetInstance().lidar_config.lidar_scan;
        LidarModel::GetInstance()->h_res_ = Degree2Radian(360.0f / static_cast<float>(lidar_horizon_scan));
        LidarModel::GetInstance()->v_res_ =
            static_cast<float>(Degree2Radian(SystemConfig::GetInstance().lidar_config.lidar_vertical_resolution));
        LidarModel::GetInstance()->lower_angle_ =
            static_cast<float>(Degree2Radian(SystemConfig::GetInstance().lidar_config.lidar_lower_angle));
    } else {
        LidarModel::GetInstance(SystemConfig::GetInstance().lidar_config.lidar_type_);
    }
}

void System::run(){
    ros::Rate rate(1000);
    while(ros::ok()){
        ros::spinOnce();
        if (!sync_package()){
            rate.sleep();
            continue;
        }
        // 处理消息
    }
}

void System::LidarCallback(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
    double current_head_time = lidar_msg->header.stamp.toSec();
    if (current_head_time < last_lidar_timestamped_){
        LOG_INFO("lidar loop back, clear buffer");
        lidar_queue_.clear();
    }
    // 将ros转换为自定义的消息类型
    auto cloud = lidar_process_->ConvertMessageToCloud(lidar_msg);
    lidar_queue_.push_back(cloud);
    lidar_time_queue_.push_back(current_head_time);
    last_lidar_timestamped_ = current_head_time;
}

// void System::LivoxLidarCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
// }

void System::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // 没有雷达消息则退出
    if (last_lidar_timestamped_ < 0){
        return ;
    }
    double imu_time = imu_msg->header.stamp.toSec();
    if (last_imu_timestamped_ > 0 && imu_time < last_imu_timestamped_){
        LOG_INFO("imu loop back, clear buffer");
        return ;
    }
    last_imu_timestamped_ = imu_time;
    IMUData imu_data;
    bool is_livox = (LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::MID360 || LidarModel::GetInstance()->lidar_sensor_type_ == LidarModel::LIDAR_TYPE::AVIA) ? true : false;
    rosIMUtoIMU(imu_msg, imu_data, is_livox, SystemConfig::GetInstance().frontend_config.axis9_imu);
    imu_queue_.push_back(imu_data);
    // fastlivo这里对imu的状态进行了传播
}

void System::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msgs){}
void System::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msgs){}
}  // namespace slam