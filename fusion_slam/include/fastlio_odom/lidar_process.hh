/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-01 23:57:31
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-06 17:47:38
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/fastlio_odom/lidar_process.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <vector>
#include "common/eigen_type.hh"
#include "common/lidar_point_type.hh"
#include "common_lib.hh"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "ikdtree/ikd_Tree.hh"
#include "sensors/lidar.hh"

namespace slam::fastlio {
struct LocalMap {
    bool initialed = false;
    BoxPointType local_map_corner;
    std::vector<BoxPointType> cub_to_rm;
};

class FastLidarProcess {
   public:
    FastLidarProcess(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf);

    void BuildLocalMap(const PointCloudPtr& cloud_world);
    void Align(const PointCloudPtr& in_cloud);

    void TrimLocalMap();
    void IncreLocalMap();
    void UpdateLossFunc(NavState& state, SharedState& shared_state);
    Mat3d GetRLtoG() {
        auto state = ieskf_->GetState();
        // r_ItoG * r_LtoI
        return state.R_ * config_.r_il;
    }

    Vec3d GetTLtoG() {
        auto state = ieskf_->GetState();
        return state.R_ * config_.t_il + state.P_;
    }

   private:
    LIONodeConfig config_;
    std::shared_ptr<FastlioIESKF> ieskf_;
    // ikdtree
    std::shared_ptr<KD_TREE<PointType>> ikdtree_ptr_;
    // local_map
    LocalMap local_map;
    // 滤波
    pcl::VoxelGrid<PointType> voxel_filter_;
    PointCloudPtr filter_cloud_lidar;
    PointCloudPtr filter_cloud_world;
    // 点面残差的法向量
    PointCloudPtr m_norm_vec;
    // 配准的点
    PointCloudPtr m_effect_cloud_lidar;
    // 有效的点面残差
    PointCloudPtr m_effect_norm_vec;
    // 点面残差有效的点
    std::vector<bool> point_selected_flag;
    std::vector<std::vector<PointType, Eigen::aligned_allocator<PointType>>> nearest_points;
};
}  // namespace slam::fastlio
