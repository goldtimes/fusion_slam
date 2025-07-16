/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-15 23:15:09
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-16 00:25:30
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/voxel_odom/voxel_odom.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <pcl/filters/voxel_grid.h>
#include "common/common_lib.hh"
#include "common/eigen_type.hh"
#include "common/logger.hh"
#include "imu_process.hh"
#include "sensors/lidar.hh"
#include "voxel_map.hh"

namespace slam {

class VoxelOdom {
   public:
    struct VoxelOdomConfig {
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
        double scan_resolution = 0.1;
        int max_layer = 5;
        double voxel_size = 0.5;
        std::vector<int> update_size_threshes = std::vector<int>{20, 10, 5, 5, 5};
        int imu_init_num = 20;
        bool extrinsic_est_en = false;
        bool align_gravity = false;
        int max_point_thresh = 100;
        int max_point_cov_thresh = 100;
        double plane_thresh = 0.01;

        double ranging_cov = 0.04;
        double angle_cov = 0.1;
    };

   public:
    VoxelOdom(const VoxelOdomConfig& config);

    void sharedUpdateFunc(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data);
    void mapping(MeasureGroup& sync_packag);

    const state_ikfom GetCurrentState() const {
        return kf_->get_x();
    }

    const SYSTEM_STATUES& GetSystemStatus() const {
        return system_status_;
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
    // PointCloudPtr cloudDownBody();
    void reset();

    PointCloudPtr transformWorld(const PointCloudPtr& cloud_in);

   private:
    M3D calcBodyCov(const V3D& point_lidar, double range_cov, double angle_cov);
    M3D transformLidarCovToWorld(const V3D& point_lidar, const M3D& cov_lidar);

   private:
    VoxelOdomConfig params;
    SYSTEM_STATUES system_status_ = SYSTEM_STATUES::INITIALIZE;
    // imu
    std::shared_ptr<IMUProcessor> imu_processor_ptr_;
    // 滤波器
    std::shared_ptr<esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>> kf_;
    // 体素滤波
    pcl::VoxelGrid<PointType> voxel_filter_;
    // voxel map
    std::unordered_map<VOXEL_KEY, OctoTree*> voxel_map_;
    // 估计外参
    bool extrinsic_est_en_;
    // 存储每帧雷达的cov
    std::vector<M3D> vars_cloud_;
    // 去畸变后的点云
    PointCloudPtr cloud_undistorted_lidar_;
    // 去畸变且降采样后的点云
    PointCloudPtr cloud_down_lidar_;
    // 世界坐标下的点云
    PointCloudPtr cloud_down_world_;
    //
    // PointCloudPtr feats_with_correspondence;
    double total_residual = 0;
    double res_mean_last = 0.0;
};
}  // namespace slam