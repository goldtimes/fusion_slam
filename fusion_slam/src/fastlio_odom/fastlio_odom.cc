/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-09 23:02:09
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-11 00:53:26
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/fastlio_odom.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "fastlio_odom/fastlio_odom.hh"
#include "common/common_lib.hh"
#include "common/eigen_type.hh"
#include "sensors/lidar.hh"
#include "sophus/so3.hpp"

namespace slam {
FastlioOdom::FastlioOdom(const FastlioOdomConfig& config) : params(config) {
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
    // 初始化KDTree
    ikdtree_ = std::make_shared<KD_TREE<PointType>>();
    ikdtree_->set_downsample_param(params.resolution);
    // 初始化点云采样器
    voxel_filter_.setLeafSize(params.resolution, params.resolution, params.resolution);

    // 初始化local_map
    local_map_.cube_len = params.cube_len;
    local_map_.det_range = params.det_range;

    extrinsic_est_en_ = params.extrinsic_est_en;

    cloud_down_lidar_.reset(new PointCloud);

    cloud_down_world_.reset(new PointCloud(20000, 1));
    norm_vec_.reset(new PointCloud(20000, 1));

    effect_cloud_lidar_.reset(new PointCloud(20000, 1));
    effect_norm_vec_.reset(new PointCloud(20000, 1));

    nearest_points_.resize(20000);
    point_select_flag_.resize(20000, false);
}
void FastlioOdom::mapping(MeasureGroup& sync_packag) {
    if (!imu_processor_ptr_->GetInitSuccess()) {
        imu_processor_ptr_->TryInit(sync_packag);
        return;
    }
    // imu传播和去畸变
    cloud_undistorted_lidar_.reset(new PointCloud);
    imu_processor_ptr_->PredictAndUndistort(sync_packag, cloud_undistorted_lidar_);
    if (system_status_ == SYSTEM_STATUES::INITIALIZE) {
        // 转换到世界坐标系下的点云
        cloud_down_world_.reset(new PointCloud);
        cloud_down_world_ = transformWorld(cloud_undistorted_lidar_);
        ikdtree_->Build(cloud_down_world_->points);
        // 构造ikdtree
        system_status_ = SYSTEM_STATUES::MAPPING;
        return;
    }
    // trimMap();
    // // align
    // double solve_H_time = 0;
    // kf_->update_iterated_dyn_share_modified(0.001, solve_H_time);
    // increaseMap();
}
void FastlioOdom::trimMap() {
    local_map_.cub_to_rm.clear();
    state_ikfom state = kf_->get_x();
    Eigen::Vector3d pos_lidar = state.pos + state.rot.toRotationMatrix() * state.offset_T_L_I;
    // 根据lidar的位置进行局部地图的初始化
    if (!local_map_.is_initialed) {
        for (int i = 0; i < 3; i++) {
            local_map_.local_map_corner.vertex_min[i] = pos_lidar[i] - local_map_.cube_len / 2.0;
            local_map_.local_map_corner.vertex_max[i] = pos_lidar[i] + local_map_.cube_len / 2.0;
        }
        local_map_.is_initialed = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = local_map_.move_thresh * local_map_.det_range;
    // 如果靠近地图边缘 则需要进行地图的移动
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - local_map_.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - local_map_.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh) need_move = true;
    }
    if (!need_move) return;

    BoxPointType new_corner, temp_corner;
    new_corner = local_map_.local_map_corner;
    float mov_dist = std::max((local_map_.cube_len - 2.0 * local_map_.move_thresh * local_map_.det_range) * 0.5 * 0.9,
                              double(local_map_.det_range * (local_map_.move_thresh - 1)));
    // 更新局部地图
    for (int i = 0; i < 3; i++) {
        temp_corner = local_map_.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh) {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = local_map_.local_map_corner.vertex_max[i] - mov_dist;
            local_map_.cub_to_rm.push_back(temp_corner);
        } else if (dist_to_map_edge[i][1] <= det_thresh) {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = local_map_.local_map_corner.vertex_min[i] + mov_dist;
            local_map_.cub_to_rm.push_back(temp_corner);
        }
    }
    local_map_.local_map_corner = new_corner;
    // 强制删除历史点云
    PointVector points_history;
    ikdtree_->acquire_removed_points(points_history);

    // 删除局部地图之外的点云
    if (local_map_.cub_to_rm.size() > 0) ikdtree_->Delete_Point_Boxes(local_map_.cub_to_rm);
    return;
}
void FastlioOdom::increaseMap() {
    if (cloud_down_lidar_->empty()) return;

    int size = cloud_down_lidar_->size();

    PointVector point_to_add;
    PointVector point_no_need_downsample;

    point_to_add.reserve(size);
    point_no_need_downsample.reserve(size);

    state_ikfom state = kf_->get_x();
    for (int i = 0; i < size; i++) {
        const PointType& p = cloud_down_lidar_->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.rot.toRotationMatrix() * (state.offset_R_L_I.toRotationMatrix() * point + state.offset_T_L_I) +
                state.pos;
        cloud_down_world_->points[i].x = point(0);
        cloud_down_world_->points[i].y = point(1);
        cloud_down_world_->points[i].z = point(2);
        cloud_down_world_->points[i].intensity = cloud_down_lidar_->points[i].intensity;
        // 如果该点附近没有近邻点则需要添加到地图中
        if (nearest_points_[i].empty()) {
            point_to_add.push_back(cloud_down_world_->points[i]);
            continue;
        }

        const PointVector& points_near = nearest_points_[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(cloud_down_world_->points[i].x / params.resolution) * params.resolution +
                      0.5 * params.resolution;
        mid_point.y = std::floor(cloud_down_world_->points[i].y / params.resolution) * params.resolution +
                      0.5 * params.resolution;
        mid_point.z = std::floor(cloud_down_world_->points[i].z / params.resolution) * params.resolution +
                      0.5 * params.resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * params.resolution &&
            fabs(points_near[0].y - mid_point.y) > 0.5 * params.resolution &&
            fabs(points_near[0].z - mid_point.z) > 0.5 * params.resolution) {
            point_no_need_downsample.push_back(cloud_down_world_->points[i]);
            continue;
        }
        float dist = sq_dist(cloud_down_world_->points[i], mid_point);

        for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < NUM_MATCH_POINTS) break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist) {
                need_add = false;
                break;
            }
        }
        if (need_add) point_to_add.push_back(cloud_down_world_->points[i]);
    }
    int add_point_size = ikdtree_->Add_Points(point_to_add, true);
    ikdtree_->Add_Points(point_no_need_downsample, false);
}

void FastlioOdom::sharedUpdateFunc(state_ikfom& state, esekfom::dyn_share_datastruct<double>& share_data) {
    int size = cloud_down_lidar_->size();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for

#endif
    for (int i = 0; i < size; i++) {
        PointType& point_body = cloud_down_lidar_->points[i];
        PointType& point_world = cloud_down_world_->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec =
            state.rot.toRotationMatrix() *
                (state.offset_R_L_I.toRotationMatrix() * point_body_vec + state.offset_T_L_I) +
            state.pos;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;

        std::vector<float> point_sq_dist(NUM_MATCH_POINTS);
        auto& points_near = nearest_points_[i];
        if (share_data.converge) {
            ikdtree_->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, point_sq_dist);
            if (points_near.size() >= NUM_MATCH_POINTS && point_sq_dist[NUM_MATCH_POINTS - 1] <= 5)
                point_select_flag_[i] = true;
            else
                point_select_flag_[i] = false;
        }
        if (!point_select_flag_[i]) continue;

        Eigen::Vector4d pabcd;
        point_select_flag_[i] = false;

        // 估计平面法向量，同时计算点面距离，计算的值存入intensity
        if (esti_plane(pabcd, points_near, 0.1)) {
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) +
                         pabcd(3);
            // 和点面距离正相关，和点的远近距离负相关
            double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            if (s > 0.9) {
                point_select_flag_[i] = true;
                norm_vec_->points[i].x = pabcd(0);
                norm_vec_->points[i].y = pabcd(1);
                norm_vec_->points[i].z = pabcd(2);
                norm_vec_->points[i].intensity = pd2;
            }
        }
    }

    int effect_feat_num = 0;
    for (int i = 0; i < size; i++) {
        if (!point_select_flag_[i]) continue;
        effect_cloud_lidar_->points[effect_feat_num] = cloud_down_lidar_->points[i];
        effect_norm_vec_->points[effect_feat_num] = norm_vec_->points[i];
        effect_feat_num++;
    }
    if (effect_feat_num < 1) {
        share_data.valid = false;
        ROS_INFO("NO Effective Points!");
        return;
    }

    share_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num, 12);
    share_data.h.resize(effect_feat_num);

    for (int i = 0; i < effect_feat_num; i++) {
        const PointType& laser_p = effect_cloud_lidar_->points[i];
        const PointType& norm_p = effect_norm_vec_->points[i];
        Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        Eigen::Vector3d temp_vec = state.offset_R_L_I.toRotationMatrix() * laser_p_vec + state.offset_T_L_I;
        Eigen::Matrix3d temp_mat;
        temp_mat << Sophus::SO3d::hat(temp_vec);
        Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.rot.toRotationMatrix() * temp_mat;
        share_data.h_x.block<1, 3>(i, 0) = norm_vec.transpose();
        share_data.h_x.block<1, 3>(i, 3) = B;
        if (extrinsic_est_en_) {
            temp_mat << Sophus::SO3d::hat(laser_p_vec);
            Eigen::Matrix<double, 1, 3> C =
                -norm_vec.transpose() * state.rot.toRotationMatrix() * state.offset_R_L_I.toRotationMatrix() * temp_mat;
            Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.rot.toRotationMatrix();
            share_data.h_x.block<1, 3>(i, 6) = C;
            share_data.h_x.block<1, 3>(i, 9) = D;
        }
        share_data.h(i) = -norm_p.intensity;
    }
}
PointCloudPtr FastlioOdom::cloudUndistortedBody() {
    PointCloudPtr cloud_undistorted_body(new PointCloud);
    Eigen::Matrix3d rot = kf_->get_x().offset_R_L_I.toRotationMatrix();
    Eigen::Vector3d pos = kf_->get_x().offset_T_L_I;
    cloud_undistorted_body->reserve(cloud_undistorted_lidar_->size());
    for (auto& p : cloud_undistorted_lidar_->points) {
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = rot * point + pos;
        PointType p_body;
        p_body.x = point(0);
        p_body.y = point(1);
        p_body.z = point(2);
        p_body.intensity = p.intensity;
        cloud_undistorted_body->points.push_back(p_body);
    }
    return cloud_undistorted_body;
}
PointCloudPtr FastlioOdom::cloudDownBody() {
    PointCloudPtr cloud_down_body(new PointCloud);
    Eigen::Matrix3d rot = kf_->get_x().offset_R_L_I.toRotationMatrix();
    Eigen::Vector3d pos = kf_->get_x().offset_T_L_I;
    cloud_down_body->reserve(cloud_down_lidar_->size());
    for (auto& p : cloud_down_lidar_->points) {
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = rot * point + pos;
        PointType p_body;
        p_body.x = point(0);
        p_body.y = point(1);
        p_body.z = point(2);
        p_body.intensity = p.intensity;
        cloud_down_body->points.push_back(p_body);
    }
    return cloud_down_body;
}

PointCloudPtr FastlioOdom::transformWorld(const PointCloudPtr& cloud_in) {
    PointCloudPtr cloud_world(new PointCloud);
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

void FastlioOdom::reset() {
    system_status_ = SYSTEM_STATUES::INITIALIZE;
    imu_processor_ptr_->reset();
    state_ikfom state = kf_->get_x();
    state.rot.setIdentity();
    state.pos.setZero();
    state.vel.setZero();
    state.offset_R_L_I.setIdentity();
    state.offset_T_L_I.setZero();
    state.ba.setZero();
    state.bg.setZero();
    kf_->change_x(state);
    ikdtree_.reset(new KD_TREE<PointType>);
}
}  // namespace slam