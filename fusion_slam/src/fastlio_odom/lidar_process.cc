/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-01 23:57:39
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-07 00:31:58
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/lidar_process.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEa
 */
#include "fastlio_odom/lidar_process.hh"
#include <Eigen/src/Core/util/Memory.h>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>
#include "common/eigen_type.hh"
#include "common/logger.hpp"
#include "common/pointcloud_utils.hh"
#include "common_lib.hh"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "sensors/lidar.hh"
#include "sophus/so3.hpp"

namespace slam::fastlio {
FastLidarProcess::FastLidarProcess(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf)
    : config_(config), ieskf_(ieskf) {
    LOG_INFO("FastLidarProcess init success!");
    ikdtree_ptr_ = std::make_shared<KD_TREE<PointType>>();
    ikdtree_ptr_->set_downsample_param(config.map_resolution);
    if (config.scan_resolution > 0.0) {
        voxel_filter_.setLeafSize(config.scan_resolution, config.scan_resolution, config.scan_resolution);
    }
    filter_cloud_lidar.reset(new PointCloud);
    filter_cloud_world.reset(new PointCloud(10000, 1));
    nearest_points.resize(20000);
    m_norm_vec.reset(new PointCloud(10000, 1));
    m_effect_cloud_lidar.reset(new PointCloud(1000, 1));
    m_effect_norm_vec.reset(new PointCloud(10000, 1));
    point_selected_flag.resize(10000);
    // 本质上是ieskf_在update函数中调用了
    // loss_func,传入当前的状态量和一些需要传递回去的变量，然后调用lidar_process的配准函数来就算H,b矩阵
    ieskf_->SetLossFunc([&](NavState& state, SharedState& shared_state) { UpdateLossFunc(state, shared_state); });
    ieskf_->SetStopFunc([&](const V21D& delta) -> bool {
        Vec3d rot_delta = delta.block<3, 1>(0, 0);
        Vec3d trans_delta = delta.block<3, 1>(3, 0);
        return (rot_delta.norm() * 57.3 < 0.01) && (trans_delta.norm() * 100 < 0.015);
    });
}
// 裁剪和移动地图
void FastLidarProcess::TrimLocalMap() {
    // 将要删除的区域清空
    local_map.cub_to_rm.clear();
    // 获取当前雷达的世界坐标位置
    auto state = ieskf_->GetState();
    // 只考虑xyz坐标
    auto pos_LinG = state.R_ * state.t_LinI + state.P_;
    if (!local_map.initialed) {
        for (int i = 0; i < 3; i++) {
            // 计算矩形框的顶点，以雷达为中心坐标
            local_map.local_map_corner.vertex_min[i] = pos_LinG[i] - config_.cube_len / 2.0;
            local_map.local_map_corner.vertex_max[i] = pos_LinG[i] + config_.cube_len / 2.0;
        }
        LOG_INFO("LocalMap INIT");
        local_map.initialed = true;
        return;
    }
    // 判断当前的位置和正方形的地图边界距离，决定local_map是否需要移动边界
    float dist_to_edge[3][2];
    bool need_move = false;
    double det_thresh = config_.move_thresh * config_.det_range;  // 90
    for (int i = 0; i < 3; i++) {
        dist_to_edge[i][0] = std::fabs(pos_LinG[i] - local_map.local_map_corner.vertex_min[i]);
        dist_to_edge[i][1] = std::fabs(pos_LinG[i] - local_map.local_map_corner.vertex_max[i]);
        if (dist_to_edge[i][0] <= det_thresh || dist_to_edge[i][1] <= det_thresh) {
            need_move = true;
            break;
        }
    }
    if (!need_move) {
        LOG_INFO("Don't need move");
        return;
    }
    // 移动local_map
    // temp_corner为移动后需要被删除的距离
    BoxPointType new_corner, temp_corner;
    // 计算移动的距离
    double move_dist = std::max((config_.cube_len - 2.0 * config_.move_thresh * config_.det_range) * 0.5 * 0.9,
                                double(config_.det_range * (config_.move_thresh - 1)));
    new_corner = local_map.local_map_corner;
    LOG_INFO("Local Map Move Dist:{}", move_dist);
    // 计算corner
    // 左移动 想象两个框一个往前移动了的框，然后多余的部分就是需要被删除的cube
    for (int i = 0; i < 3; i++) {
        temp_corner = local_map.local_map_corner;
        if (dist_to_edge[i][0] <= det_thresh) {
            new_corner.vertex_min[i] -= move_dist;
            new_corner.vertex_max[i] -= move_dist;
            temp_corner.vertex_min[i] = temp_corner.vertex_max[i] - move_dist;
            local_map.cub_to_rm.push_back(temp_corner);
        } else if (dist_to_edge[i][1] <= det_thresh) {
            new_corner.vertex_min[i] += move_dist;
            new_corner.vertex_max[i] += move_dist;
            temp_corner.vertex_max[i] = temp_corner.vertex_min[i] + move_dist;
            local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    // 更新local_map的角点
    local_map.local_map_corner = new_corner;
    std::vector<PointType, Eigen::aligned_allocator<PointType>> points_history;
    ikdtree_ptr_->acquire_removed_points(points_history);
    // 删除局部地图之外的点云
    if (local_map.cub_to_rm.size() > 0) {
        ikdtree_ptr_->Delete_Point_Boxes(local_map.cub_to_rm);
    }
    return;
}
// 增加局部地图
void FastLidarProcess::IncreLocalMap() {
    // 点云为空
    LOG_INFO("IncreLocalMap");
    if (filter_cloud_lidar->empty()) {
        return;
    }

    // 当前状态
    const NavState& state = ieskf_->GetState();
    int cloud_size = filter_cloud_lidar->size();
    std::vector<PointType, Eigen::aligned_allocator<PointType>> point_to_add;
    std::vector<PointType, Eigen::aligned_allocator<PointType>> point_to_add_downsample;
    for (int i = 0; i < cloud_size; ++i) {
        const PointType& point_lidar = filter_cloud_lidar->points[i];
        const Vec3d& point_lidar_vec = Vec3d(point_lidar.x, point_lidar.y, point_lidar.z);
        const auto point_world_vec =
            ieskf_->GetState().R_ * (ieskf_->GetState().R_LtoI * point_lidar_vec + ieskf_->GetState().t_LinI) +
            ieskf_->GetState().P_;
        filter_cloud_world->points[i].x = point_world_vec[0];
        filter_cloud_world->points[i].y = point_world_vec[1];
        filter_cloud_world->points[i].z = point_world_vec[2];
        filter_cloud_world->points[i].intensity = point_lidar.intensity;
        // 如果kdtree没有搜索到最近邻
        if (nearest_points[i].empty()) {
            point_to_add.push_back(filter_cloud_world->points[i]);
            continue;
        }
        // 最近邻的点
        std::vector<PointType, Eigen::aligned_allocator<PointType>> points_near = nearest_points[i];
        bool need_add = true;
        // 体素化点
        PointType mid_point, downsample_result;
        Eigen::Vector3f voxel_index(std::floor(filter_cloud_world->points[i].x / config_.map_resolution),
                                    std::floor(filter_cloud_world->points[i].y / config_.map_resolution),
                                    std::floor(filter_cloud_world->points[i].z / config_.map_resolution));
        mid_point.x = voxel_index[0] * config_.map_resolution + 0.5 * config_.map_resolution;
        mid_point.y = voxel_index[1] * config_.map_resolution + 0.5 * config_.map_resolution;
        mid_point.z = voxel_index[2] * config_.map_resolution + 0.5 * config_.map_resolution;
        // 如果最近的邻近点都在体素外面，则不需要降采样就加入
        if (std::fabs(points_near[0].x - mid_point.x) > 0.5 * config_.map_resolution &&
            std::fabs(points_near[0].y - mid_point.y) > 0.5 * config_.map_resolution &&
            std::fabs(points_near[0].z - mid_point.z) > 0.5 * config_.map_resolution) {
            point_to_add_downsample.push_back(filter_cloud_world->points[i]);
            continue;
        }
        float point_world_to_voxel_center_dist = sq_dist(filter_cloud_world->points[i], mid_point);
        // 遍历最邻近的点
        for (int j = 0; j < config_.near_search_num; ++j) {
            // 不够5个点
            if (points_near.size() < static_cast<size_t>(config_.near_search_num)) {
                break;
            }
            if (sq_dist(points_near[j], mid_point) < point_world_to_voxel_center_dist) {
                need_add = false;
                break;
            }
        }
        if (need_add) {
            point_to_add.push_back(filter_cloud_world->points[i]);
        }
        ikdtree_ptr_->Add_Points(point_to_add, true);
        ikdtree_ptr_->Add_Points(point_to_add_downsample, false);
    }
}

void FastLidarProcess::BuildLocalMap(const PointCloudPtr& cloud_world) {
    // 构建ikdtree
    ikdtree_ptr_->Build(cloud_world->points);
}

// 点面残差的构建
void FastLidarProcess::UpdateLossFunc(NavState& state, SharedState& shared_state) {
    size_t size_num = filter_cloud_lidar->size();
    LOG_INFO("input cloud size:{}, nearest_points:{}", size_num, nearest_points.size());
    filter_cloud_world->resize(size_num);
#ifdef MP_EN
    omp_set_num_threads(2);
#pragma omp parallel for
#endif
    for (int i = 0; i < size_num; ++i) {
        const PointType& point_body = filter_cloud_lidar->points[i];
        PointType& point_world = filter_cloud_world->points[i];
        const auto point_body_vec = Vec3d(point_body.x, point_body.y, point_body.z);
        const auto point_world_vec =
            ieskf_->GetState().R_ * (ieskf_->GetState().R_LtoI * point_body_vec + ieskf_->GetState().t_LinI) +
            ieskf_->GetState().P_;
        point_world.x = point_world_vec.x();
        point_world.y = point_world_vec.y();
        point_world.z = point_world_vec.z();
        point_world.intensity = point_body.intensity;
        // 最近5个点的距离
        std::vector<float> dist2(config_.near_search_num);
        // 最近的5个点
        std::vector<PointType, Eigen::aligned_allocator<PointType>>& points_nears = nearest_points[i];
        LOG_INFO("points_nears:{}", points_nears.size());
        LOG_INFO("i:{}", i);
        ikdtree_ptr_->Nearest_Search(point_world, config_.near_search_num, points_nears, dist2);
        LOG_INFO("Nearest_Search:{}", points_nears.size());

        if (points_nears.size() >= static_cast<size_t>(config_.near_search_num)) {
            point_selected_flag[i] = true;
        } else {
            point_selected_flag[i] = false;
        }
        if (!point_selected_flag[i]) {
            continue;
        }
        LOG_INFO("esti_plane");
        // // 估计平面
        Eigen::Vector4d plane_coeff;
        point_selected_flag[i] = false;
        if (esti_plane(points_nears, 0.1, plane_coeff)) {
            double point_to_plane_dist2 = plane_coeff(0) * point_world_vec(0) + plane_coeff(1) * point_world_vec(1) +
                                          plane_coeff(2) * point_world_vec(2) + plane_coeff(3);
            //远处的点需要更严格的平面距离约束
            // 分母为点的距离，点的距离越远，只有当point_to_plane_dist2越小时，才能满足s
            //> 0.9的约束
            double s = 1 - 0.9 * std::fabs(point_to_plane_dist2) / std::sqrt(point_body_vec.norm());
            if (s > 0.9) {
                point_selected_flag[i] = true;
                m_norm_vec->points[i].x = plane_coeff(0);
                m_norm_vec->points[i].y = plane_coeff(1);
                m_norm_vec->points[i].z = plane_coeff(2);
                m_norm_vec->points[i].intensity = point_to_plane_dist2;
            }
        }
    }
    int num_effect_size = 0;
    for (int i = 0; i < point_selected_flag.size(); ++i) {
        if (!point_selected_flag[i]) {
            continue;
        }
        m_effect_cloud_lidar->points[num_effect_size] = filter_cloud_lidar->points[i];
        m_effect_norm_vec->points[num_effect_size] = m_norm_vec->points[i];
        num_effect_size++;
    }
    if (num_effect_size < 1) {
        shared_state.valid = false;
        LOG_INFO("Find Num Effect size < 1");
        return;
    }
    LOG_INFO("Find Effect size:{}", num_effect_size);
    shared_state.valid = true;
    shared_state.H.setZero();
    shared_state.b.setZero();
    // 点面距离的旋转和平移的雅可比矩阵以及对外参的雅可比矩阵所有是1，12维度
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < num_effect_size; ++i) {
        J.setZero();
        const PointType& lidar_point = m_effect_cloud_lidar->points[i];
        const PointType& norm_point = m_effect_norm_vec->points[i];
        const Vec3d lidar_point_vec(lidar_point.x, lidar_point.y, lidar_point.z);
        const Vec3d lidar_body_vec = ieskf_->GetState().R_LtoI * lidar_point_vec + ieskf_->GetState().t_LinI;
        const Vec3d norm_vec(norm_point.x, norm_point.y, norm_point.z);
        // 对旋转的雅可比矩阵
        Eigen::Matrix<double, 1, 3> B =
            -norm_vec.transpose() * ieskf_->GetState().R_ * Sophus::SO3d::hat(lidar_body_vec);
        J.block<1, 3>(0, 0) = B;
        J.block<1, 3>(0, 3) = norm_vec.transpose();
        if (config_.esti_il) {
            Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * ieskf_->GetState().R_ * ieskf_->GetState().R_LtoI *
                                            Sophus::SO3d::hat(lidar_point_vec);
            Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * ieskf_->GetState().R_;
            J.block<1, 3>(0, 6) = C;
            J.block<1, 3>(0, 9) = D;
        }
        shared_state.H += J.transpose() * config_.lidar_cov_inv * J;
        // 这里是不是有问题
        shared_state.b += J.transpose() * config_.lidar_cov_inv * norm_point.intensity;
    }
    LOG_INFO("UpdateLossFunc end");
}

void FastLidarProcess::Align(const PointCloudPtr& in_cloud) {
    if (config_.scan_resolution > 0.0) {
        voxel_filter_.setInputCloud(in_cloud);
        voxel_filter_.filter(*filter_cloud_lidar);
    } else {
        filter_cloud_lidar = in_cloud;
    }
    LOG_INFO("after filter size:{}", filter_cloud_lidar->size());
    TrimLocalMap();
    // align
    ieskf_->Update();
    // IncreLocalMap();
}
}  // namespace slam::fastlio