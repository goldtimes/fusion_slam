/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-01 23:58:44
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-03 00:24:00
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/fastlio_odom/fastlio_odom.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <memory>
#include "common_lib.hh"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "fastlio_odom/imu_process.hh"
#include "fastlio_odom/lidar_process.hh"
namespace slam::fastlio {

enum class ODOM_STATE {
    IMU_INIT,
    MAP_INIT,
    MAPPING,
};

class FastLioOdom {
   public:
    FastLioOdom(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf);
    ~FastLioOdom() = default;

    const std::shared_ptr<FastLidarProcess>& GetLidarProcess() const {
        return lidar_process_ptr_;
    }

    const ODOM_STATE& GetState() const {
        return odom_state;
    }

    void ProcessSyncpackage(MeasureGroup& measure);

   private:
    LIONodeConfig config_;
    ODOM_STATE odom_state = ODOM_STATE::IMU_INIT;
    std::shared_ptr<FastlioIESKF> ieskf_;
    std::shared_ptr<IMUProcess> imu_process_ptr_;
    std::shared_ptr<FastLidarProcess> lidar_process_ptr_;
};
}  // namespace slam::fastlio