#include "System.hh"
#include <string>
#include "SystemConfig.hh"
#include "common/PoseTrans.hpp"
#include "common/logger.hpp"

namespace slam {
System::System(const ros::NodeHandle& nh) : nh_(nh) {
    // 初始化配置文件
    InitConfig();
}
void System::InitConfig() {
    SystemConfig& config = SystemConfig::GetInstance();

    // sensor topic name
    nh_.getParam("sensor_topic/lidar_topic", config.lidar_topic);
    nh_.getParam("sensor_topic/imu_topic", config.imu_topic);

    int temp;
    // nh_.param("slam_mode", temp, 0);
    // slam_mode_ = static_cast<SLAM_MODE>(temp);

    // lidar config parameters
    std::string lidar_type;
    nh_.param<std::string>("lidar/lidar_sensor_type", lidar_type, "");
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

    // calibration parameters
    std::vector<double> lidar_to_imu;
    nh_.getParam("calibration/lidar_to_imu", lidar_to_imu);
    Eigen::Matrix4d T_I_L = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(lidar_to_imu.data());
    config.T_I_L = PoseTranseD(T_I_L.block<3, 3>(0, 0), T_I_L.block<3, 1>(0, 3));
    LOG_INFO("T_I_L:\n{}", T_I_L);
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
}  // namespace slam