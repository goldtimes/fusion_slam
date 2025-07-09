#include "fastlio_odom/fastlio_odom.hh"

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
    imu_processor_ptr_->setExtParams(config.imu_ext_rot, config.imu_ext_pos);
    // imu_processor_ptr_->setCov(params.imu_gyro_cov, params.imu_acc_cov, params.imu_gyro_bias_cov,
    //                            params.imu_acc_bias_cov);
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
    if (imu_processor_ptr_->TryInit(sync_packag)) {
        system_status_ = SYSTEM_STATUES::MAPPING;
    }
}
void FastlioOdom::trimMap() {
}
void FastlioOdom::increaseMap() {
}

void FastlioOdom::sharedUpdateFunc(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data) {
}
PointCloudPtr FastlioOdom::cloudUndistortedBody() {
}
PointCloudPtr FastlioOdom::cloudDownBody() {
}
}  // namespace slam