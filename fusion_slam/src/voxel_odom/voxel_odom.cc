/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-15 23:13:59
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-16 00:47:59
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/voxel_odom/voxel_odom.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "voxel_odom/voxel_odom.hh"
#include <pcl/pcl_macros.h>
#include <cmath>
#include <vector>
#include "common/common_lib.hh"
#include "sensors/lidar.hh"
#include "voxel_odom/voxel_map.hh"

namespace slam {
VoxelOdom::VoxelOdom(const VoxelOdomConfig& config) {
    // 初始化ESIKF
    kf_ = std::make_shared<esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>>();
    std::vector<double> epsi(23, 0.001);
    kf_->init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) { sharedUpdateFunc(s, ekfom_data); },
        params.esikf_max_iteration, epsi.data());

    // 初始化IMUProcessor
    IMUProcessor::IMUProcessorConfig imu_process_config_;
    imu_process_config_.align_gravity = config.align_gravity;
    imu_processor_ptr_ = std::make_shared<IMUProcessor>(imu_process_config_, kf_);
    // 设置外参
    imu_processor_ptr_->setExtParams(config.imu_ext_rot, config.imu_ext_pos);
    // 设置Q_过程噪声
    imu_processor_ptr_->SetCov(params.imu_gyro_cov, params.imu_acc_cov, params.imu_gyro_bias_cov,
                               params.imu_acc_bias_cov);

    // 初始化点云采样器
    voxel_filter_.setLeafSize(params.scan_resolution, params.scan_resolution, params.scan_resolution);

    extrinsic_est_en_ = params.extrinsic_est_en;

    cloud_down_lidar_.reset(new PointCloud);
    cloud_undistorted_lidar_.reset(new PointCloud);
    cloud_down_world_.reset(new PointCloud);
}

void VoxelOdom::mapping(MeasureGroup& sync_packag) {
    // imu 初始化
    if (!imu_processor_ptr_->GetInitSuccess()) {
        imu_processor_ptr_->TryInit(sync_packag);
        return;
    }
    // imu传播和去畸变
    cloud_undistorted_lidar_.reset(new PointCloud);
    imu_processor_ptr_->PredictAndUndistort(sync_packag, cloud_undistorted_lidar_);
    // 滤波
    voxel_filter_.setInputCloud(cloud_undistorted_lidar_);
    voxel_filter_.filter(*cloud_down_lidar_);
    if (system_status_ == SYSTEM_STATUES::INITIALIZE) {
        // 转换点云到世界坐标系下
        cloud_down_world_ = transformWorld(cloud_down_lidar_);
        std::vector<PointWithCov> pv_list;
        for (int i = 0; i < cloud_down_world_->size(); ++i) {
            PointWithCov pv;
            V3D point_lidar(cloud_down_lidar_->points[i].x, cloud_down_lidar_->points[i].y,
                            cloud_down_lidar_->points[i].z);
            V3D point_world(cloud_down_world_->points[i].x, cloud_down_world_->points[i].y,
                            cloud_down_world_->points[i].z);
            pv.point = point_lidar;
            pv.point_wolrd = point_world;
            // 防止为0
            if (point_lidar[2] == 0.0) {
                point_lidar[2] = 0.001;
            }
            M3D cov_lidar = calcBodyCov(point_lidar, params.ranging_cov, params.angle_cov);
            // 计算世界坐标下的cov
            M3D cov_world = transformLidarCovToWorld(point_lidar, cov_lidar);
            pv.cov_lidar = cov_lidar;
            pv.cov = cov_world;
            pv_list.push_back(pv);
        }
        // 构建第一帧的voxel_map
        buildVoxelMap(pv_list, params.voxel_size, params.max_layer, params.update_size_threshes,
                      params.max_point_thresh, params.max_point_cov_thresh, params.plane_thresh, voxel_map_);
        LOG_INFO("Build Voxel Map Size:{}", pv_list.size());
        system_status_ = SYSTEM_STATUES::MAPPING;
        return;
    }
    // align
    std::sort(cloud_down_lidar_->begin(), cloud_down_lidar_->end(),
              [](const PointType& p1, const PointType& p2) { return p1.curvature < p2.curvature; });
    auto point_size = cloud_down_lidar_->size();
    if (point_size < 5) {
        LOG_ERROR("No point, skip this scan");
    }
    // 计算lidar每个点的协方差

    vars_cloud_.clear();
    vars_cloud_.reserve(point_size);
    for (auto& pt : cloud_down_lidar_->points) {
        V3D point_lidar(pt.x, pt.y, pt.z);
        M3D point_cov = calcBodyCov(point_lidar, params.ranging_cov, params.angle_cov);
        vars_cloud_.push_back(point_cov);
    }
    double solve_H_time;
    auto start = std::chrono::high_resolution_clock::now();
    kf_->update_iterated_dyn_share_modified(0.001, solve_H_time);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOG_INFO("Align By Voxel used time:{}", duration.count() * 1e-3);
    // print state
    // Update Voxel map
    std::vector<PointWithCov> pv_world_list;
    pv_world_list.reserve(cloud_down_lidar_->size());
    // 根据最新的状态，将点云转到world系
    PointCloudPtr cloud_world = transformWorld(cloud_down_lidar_);
    for (size_t i = 0; i < cloud_world->size(); ++i) {
        PointWithCov pv;
        V3D point_lidar(cloud_down_lidar_->points[i].x, cloud_down_lidar_->points[i].y, cloud_down_lidar_->points[i].z);
        V3D point_wolrd(cloud_world->points[i].x, cloud_world->points[i].y, cloud_world->points[i].z);

        M3D point_lidar_cov = vars_cloud_[i];
        M3D point_world_cov = transformLidarCovToWorld(point_lidar, point_lidar);
        pv.point = point_lidar;
        pv.point_wolrd = point_wolrd;
        pv.cov_lidar = point_lidar_cov;
        pv.cov = point_world_cov;
        pv_world_list.push_back(pv);
    }
    start = std::chrono::high_resolution_clock::now();
    std::sort(pv_world_list.begin(), pv_world_list.end(), []() {});
    updateVoxelMapOMP(pv_world_list, params.voxel_size, params.max_layer, params.update_size_threshes,
                      params.max_point_thresh, params.max_point_cov_thresh, params.plane_thresh, voxel_map_);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOG_INFO("Update Voxel Map", duration.count() * 1e-3);
}

M3D VoxelOdom::calcBodyCov(const V3D& point_lidar, double range_inc, double angle_inc) {
    // 距离
    double range =
        sqrt(point_lidar[0] * point_lidar[0] + point_lidar[1] * point_lidar[1] + point_lidar[2] * point_lidar[2]);
    // 距离噪声
    double range_var = range_inc * range_inc;
    // 方向噪声
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(angle_inc)), 2), 0, 0, pow(sin(DEG2RAD(angle_inc)), 2);
    V3D direction(point_lidar);
    // 归一化
    direction.normalize();
    M3D direction_hat;
    // clang-format off
    direction_hat << 0, -direction(2), direction(1),
                    direction(2), 0, -direction(0),
                    -direction(1), direction(2), 0;
    // clang-format on
    V3D base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();

    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    return direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

M3D VoxelOdom::transformLidarCovToWorld(const V3D& point_lidar, const M3D& cov_lidar) {
    M3D point_hat;
    point_hat << SKEW_SYM_MATRX(point_lidar);
    auto state = kf_->get_x();
    M3D R_IL_cov = kf_->get_P().block<3, 3>(6, 6);
    M3D t_IL_cov = kf_->get_P().block<3, 3>(9, 9);
    M3D cov_body = state.offset_R_L_I * cov_lidar * state.offset_R_L_I.conjugate() +
                   state.offset_R_L_I * (-point_hat) * R_IL_cov * (-point_hat).transpose() + t_IL_cov;
    // 转到到body坐标系
    V3D point_body = state.offset_R_L_I * point_lidar + state.offset_T_L_I;

    // 计算世界坐标系下的cov;
    point_hat << SKEW_SYM_MATRX(point_body);
    M3D rot_var = kf_->get_P().block<3, 3>(3, 3);
    M3D trans_var = kf_->get_P().block<3, 3>(0, 0);
    M3D cov_world = state.rot * cov_body * state.rot.conjugate() +
                    state.rot * (-point_hat) * rot_var * (-point_hat).transpose() * state.rot.conjugate() + trans_var;
    return cov_world;
}

void VoxelOdom::sharedUpdateFunc(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data) {
}

PointCloudPtr VoxelOdom::transformWorld(const PointCloudPtr& cloud_in) {
    PointCloudPtr cloud_world(new PointCloud);
    if (cloud_in->empty()) {
        return cloud_world;
    }
    cloud_world->reserve(cloud_in->size());
    // 当前的状态
    M3D R_wi = kf_->get_x().rot.matrix();
    V3D p_wi = kf_->get_x().pos;
    M3D r_il = kf_->get_x().offset_R_L_I.matrix();
    V3D t_il = kf_->get_x().offset_T_L_I;
    for (int i = 0; i < cloud_in->size(); ++i) {
        auto point_lidar_eigen = PointToEigen(cloud_in->points[i]);
        auto point_world_eigen = R_wi * (r_il * point_lidar_eigen + t_il) + p_wi;
        auto point_world = EigenToPoint(point_world_eigen);
        point_world.intensity = cloud_in->points[i].intensity;
        point_world.curvature = cloud_in->points[i].curvature;
        cloud_world->push_back(point_world);
    }
    return cloud_world;
}

}  // namespace slam