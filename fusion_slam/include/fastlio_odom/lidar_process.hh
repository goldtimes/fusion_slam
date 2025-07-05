/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-01 23:57:31
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-05 12:12:50
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/fastlio_odom/lidar_process.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <memory>
#include "common/eigen_type.hh"
#include "common_lib.hh"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "lidar_process.hh"
#include "sensors/lidar.hh"
namespace slam::fastlio {
class FastLidarProcess {
   public:
    FastLidarProcess(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf);

    void BuildLocalMap(const PointCloudPtr& cloud_world);
    void Align(const PointCloudPtr& in_cloud);
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
};
}  // namespace slam::fastlio
