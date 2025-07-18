#pragma once
#include <fstream>
#include <memory>
#include <ostream>
#include <vector>
#include "common/PoseTrans.hpp"
#include "common/navi_state.hh"
#include "common_lib.hh"
#include "lio-ieskf/ieskf.hh"
#include "sensors/encoder.hh"
#include "sensors/gnss.hh"
#include "sensors/imu.hh"
#include "sensors/lidar.hh"
#include "static_imu_init.hh"

/**
    imu的前向传播,在这里也需要完成轮速计的更新和gnss的更新
 */
namespace slam {
class ImuProgator {
   public:
    ImuProgator(const std::shared_ptr<IESKF>& ieskf);
    ~ImuProgator();

    void set_lidar_extrinsic(const PoseTranseD& T_LtoI) {
        T_LtoI_ = T_LtoI;
    }

    void set_encoder_extrinsic(const PoseTranseD& T_EtoI) {
        T_EtoI_ = T_EtoI;
    }

    void set_gps_extrinsic(const PoseTranseD& T_GPStoI) {
        T_GPStoI_ = T_GPStoI;
    }

    void Process(const MeasureGroup& measures, NaviState& state, PointCloudPtr& pcl_out);
    void Propagation(const MeasureGroup& measures, NaviState& state, PointCloudPtr& pcl_out);

   private:
    // 写文件
    std::ofstream state_ofstream;
    // imu初始化器
    std::shared_ptr<StateicImuInit> static_imu_init_;
    // 是否初始化了
    bool imu_inited_ = false;
    // 雷达外参
    PoseTranseD T_LtoI_;
    // 轮速计的外参
    PoseTranseD T_EtoI_;
    // gnss的外参
    PoseTranseD T_GPStoI_;

    IMUData last_imu_data;
    double last_lidar_end_time = -1;
    bool gnss_inited_ = false;

    // ieskf
    std::shared_ptr<IESKF> ieskf_ptr_;
    std::vector<NaviState> imu_states_;
};
}  // namespace slam