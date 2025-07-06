/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-01 23:57:39
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-06 17:19:37
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/fastlio_odom/lidar_process.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEa
 */
#include "fastlio_odom/lidar_process.hh"
#include <cmath>
#include <memory>
#include <vector>
#include "common/eigen_type.hh"
#include "common/logger.hpp"
#include "common_lib.hh"
#include "sensors/lidar.hh"

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
    filter_cloud_world.reset(new PointCloud);
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
    if (filter_cloud_lidar->empty()) {
        return;
    }
}

void FastLidarProcess::BuildLocalMap(const PointCloudPtr& cloud_world) {
    // 构建ikdtree
    ikdtree_ptr_->Build(cloud_world->points);
}

// 点面残差的构建
void FastLidarProcess::UpdateLossFunc(NavState& state, SharedState& shared_state) {
    int size_num = filter_cloud_lidar->size();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size_num; ++i) {
    }
}

void FastLidarProcess::Align(const PointCloudPtr& in_cloud) {
    if (config_.scan_resolution > 0.0) {
        voxel_filter_.setInputCloud(in_cloud);
        voxel_filter_.filter(*filter_cloud_lidar);
    } else {
        filter_cloud_lidar = in_cloud;
        // in_cloud.reset(new PointCloud());
    }
    TrimLocalMap();
    // align
    ieskf_->Update();
    IncreLocalMap();
}
}  // namespace slam::fastlio