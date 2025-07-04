/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-01 23:57:02
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-03 00:01:39
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/fastlio_odom/imu_process.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <memory>
#include <vector>
#include "common/eigen_type.hh"
#include "common/logger.hpp"
#include "common_lib.hh"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "sensors/imu.hh"
#include "sensors/lidar.hh"
#include "static_imu_init.hh"
namespace slam::fastlio {
class IMUProcess {
   public:
    struct IMUState {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        IMUState() {
        }
        IMUState(double t, const Mat3d& r, const Vec3d& p, const Vec3d& v, const Vec3d& a, const Vec3d& g)
            : offset_time(t), rot(r), pos(p), vel(v), acc(a), gyro(g) {
        }
        double offset_time;
        Mat3d rot;
        Vec3d pos;
        Vec3d vel;
        Vec3d acc;
        Vec3d gyro;
    };

    IMUProcess(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf);

    bool TrytoInit(const MeasureGroup& measure);

    void PredictAndUndistort(const MeasureGroup& measure, PointCloud::Ptr& undistort_cloud);

   private:
    LIONodeConfig config_;
    std::shared_ptr<FastlioIESKF> ieskf_;
    std::shared_ptr<StateicImuInit> imu_init_ptr_;
    std::vector<IMUState> imu_states_;
    std::vector<IMUData> imu_queue_;
    double m_last_propagate_end_time;
    Vec3d last_acc_;
    Vec3d last_gyro_;
    IMUData last_imu_data_;
    // imu的噪声矩阵
    M12D Q_;
};
}  // namespace slam::fastlio