#pragma once
#include <memory>
#include "common/common_lib.hh"

namespace slam {
class StaticImuInit;
}  // namespace slam
namespace slam {

struct Pose {
    Pose();
    Pose(double t, V3D a, V3D g, V3D v, V3D p, M3D r) : offset(t), acc(a), gyro(g), vel(v), pos(p), rot(r) {
    }
    double offset;
    V3D acc;
    V3D gyro;
    M3D rot;
    V3D pos;
    V3D vel;
};
class IMUProcessor {
   public:
    struct IMUProcessorConfig {
        bool align_gravity;
    };
    IMUProcessor(const IMUProcessorConfig& config,
                 std::shared_ptr<esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>> kf);

    bool TryInit(MeasureGroup& sync_package);

    void PredictAndUndistort(MeasureGroup& sync_package, PointCloudPtr& undistort_pcl);

    void setExtParams(const M3D& rot, const V3D& pos);

    void reset();

   private:
    IMUProcessorConfig config_;
    std::shared_ptr<esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>> kf_;
    std::shared_ptr<StaticImuInit> static_imu_init_;
    // 外参信息
    M3D rot_ext;
    V3D pos_ext;
    double last_lidar_time_end_;
    // ieskf的过程噪声
    Eigen::Matrix<double, 12, 12> Q_;
    // 用来去畸变的imu pose
    std::vector<Pose> undistored_imu_poses;
    IMUData last_imu_data_;
};
}  // namespace slam