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
    FastLidarProcess(const LIONodeConfig &config, std::shared_ptr<FastlioIESKF> kf);

    void trimCloudMap();

    void incrCloudMap();

    void initCloudMap(PointVec &point_vec);

    void process(MeasureGroup &package);

    void updateLossFunc(NavState &state, SharedState &share_data);
    Mat3d GetRLtoG() {
        auto state = m_kf->GetState();
        // r_ItoG * r_LtoI
        return state.R_ * m_config.r_il;
    }

    Vec3d GetTLtoG() {
        auto state = m_kf->GetState();
        return state.R_ * m_config.t_il + state.P_;
    }

   private:
    LIONodeConfig m_config;
    LocalMap m_local_map;
    std::shared_ptr<FastlioIESKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;
    PointCloudPtr m_cloud_lidar;
    PointCloudPtr m_cloud_down_lidar;
    PointCloudPtr m_cloud_down_world;
    std::vector<bool> m_point_selected_flag;
    PointCloudPtr m_norm_vec;
    PointCloudPtr m_effect_cloud_lidar;
    PointCloudPtr m_effect_norm_vec;
    std::vector<PointVec> m_nearest_points;
    pcl::VoxelGrid<PointType> m_scan_filter;
};
}  // namespace slam::fastlio
