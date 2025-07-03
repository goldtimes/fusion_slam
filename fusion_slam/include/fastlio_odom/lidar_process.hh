/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-01 23:57:31
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-03 00:23:53
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/fastlio_odom/lidar_process.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <memory>
#include "common_lib.hh"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "lidar_process.hh"
namespace slam::fastlio {
class FastLidarProcess {
   public:
    FastLidarProcess(const LIONodeConfig& config, std::shared_ptr<FastlioIESKF> ieskf);

   private:
    LIONodeConfig config_;
    std::shared_ptr<FastlioIESKF> ieskf_;
};
}  // namespace slam::fastlio
