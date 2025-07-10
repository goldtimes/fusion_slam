#pragma once
#include <memory>
#include "common/common_lib.hh"
#include "common/eigen_type.hh"

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

    void SetCov(double gyro_cov, double acc_cov, double gyro_bias_cov, double acc_bias_cov);
    void SetCov(const V3D& gyro_cov, const V3D& acc_cov, const V3D& gyro_bias_cov, const V3D& acc_bias_cov);
    void Undistort_cloud(PointCloudPtr& out);
    void reset();

    const bool& GetInitSuccess() const {
        return init_success_;
    }

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
    int init_imu_count;
    V3D mean_acc, mean_gyro;
    bool init_success_ = false;
    // imu来传播的队列
    std::deque<IMUData> imus;
    // 存储用来去畸变的imu_pose
    std::vector<Pose> imu_poses_;
    V3D last_acc;
    V3D last_gyro;
    V3D gyro_cov_;
    V3D acc_cov_;
    V3D acc_bias_cov_;
    V3D gyro_bias_cov_;
};
}  // namespace slam