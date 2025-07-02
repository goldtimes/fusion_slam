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
#include "common/logger.hpp"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "static_imu_init.hh"
namespace slam::fastlio {
class IMUProcess {
   public:
    struct IMUProcessConfig {};
    IMUProcess(std::shared_ptr<FastlioIESKF> ieskf, const IMUProcessConfig& config);

   private:
    IMUProcessConfig imu_process_config_;
    std::shared_ptr<FastlioIESKF> ieskf_;
    std::shared_ptr<StateicImuInit> imu_init_ptr_;
};
}  // namespace slam::fastlio