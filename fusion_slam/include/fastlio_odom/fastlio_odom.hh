#pragma once
#include <pcl/filters/voxel_grid.h>
#include "common/common_lib.hh"
#include "common/logger.hh"
#include "fastlio_odom/ikd-Tree/ikd_Tree.h"
#include "imu_process.hh"
#include "sensors/lidar.hh"

namespace slam {
class IMUProcessor;
}

namespace slam {

enum class SYSTEM_STATUES {
    INITIALIZE,
    RELOCALIZATION,
    LOCALIZATION,
    MAPPING,
};

struct LocalMap {
    double cube_len;
    double det_range;
    double move_thresh;
    BoxPointType local_map_corner;
    std::vector<BoxPointType> cub_to_rm;
    bool is_initialed;
};

class FastlioOdom {
   public:
    struct FastlioOdomConfig {
        // ikdtree的分辨率
        double resolution = 0.1;
        double esikf_min_iteration = 2;
        double esikf_max_iteration = 5;
        // 白噪声
        double imu_acc_cov = 0.01;
        double imu_gyro_cov = 0.01;
        // 零偏噪声
        double imu_acc_bias_cov = 0.0001;
        double imu_gyro_bias_cov = 0.0001;
        // 外参信息
        M3D imu_ext_rot;
        V3D imu_ext_pos;
        // std::vector<double> imu_ext_rot = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        // std::vector<double> imu_ext_pos = {-0.011, -0.02329, 0.04412};
        // local_map移动相关
        double cube_len = 1000.0;
        double det_range = 100.0;
        double move_thresh = 1.5;
        bool extrinsic_est_en = false;
        bool align_gravity = false;
    };

   public:
    FastlioOdom(const FastlioOdomConfig& config);

    void sharedUpdateFunc(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data);
    void mapping(MeasureGroup& sync_packag);
    void trimMap();
    void increaseMap();
    // Getter

    const state_ikfom GetCurrentState() const {
        return kf_->get_x();
    }

    const SYSTEM_STATUES& GetSystemStatus() const {
        return system_status_;
    }
    std::shared_ptr<KD_TREE<PointType>> getIKDtree() {
        return ikdtree_;
    }
    std::shared_ptr<esekfom::esekf<state_ikfom, 12, input_ikfom>> getKF() {
        return kf_;
    }
    std::shared_ptr<IMUProcessor> getIMUProcessor() {
        return imu_processor_ptr_;
    }
    PointCloudPtr GetcCloudWorld() {
        return cloud_down_world_;
    }
    PointCloudPtr GetCloudLidar() {
        return cloud_down_lidar_;
    }
    PointCloudPtr cloudUndistortedLidar() {
        return cloud_undistorted_lidar_;
    }
    PointCloudPtr cloudUndistortedBody();
    PointCloudPtr cloudDownBody();
    void reset();

    PointCloudPtr transformWorld(const PointCloudPtr& cloud_in);

   private:
    FastlioOdomConfig params;
    SYSTEM_STATUES system_status_ = SYSTEM_STATUES::INITIALIZE;
    // imu
    std::shared_ptr<IMUProcessor> imu_processor_ptr_;
    // 滤波器
    std::shared_ptr<esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>> kf_;
    // ikdtree
    std::shared_ptr<KD_TREE<PointType>> ikdtree_;
    // 体素滤波
    pcl::VoxelGrid<PointType> voxel_filter_;
    // 地图
    LocalMap local_map_;
    // 估计外参
    bool extrinsic_est_en_;
    // 点云
    PointCloudPtr cloud_down_lidar_;
    PointCloudPtr cloud_undistorted_lidar_;
    PointCloudPtr cloud_down_world_;
    // 搜索临近的点
    std::vector<PointVector> nearest_points_;
    std::vector<bool> point_select_flag_;
    // 有效的法向量
    PointCloudPtr norm_vec_;
    PointCloudPtr effect_norm_vec_;
    PointCloudPtr effect_cloud_lidar_;
};
}  // namespace slam