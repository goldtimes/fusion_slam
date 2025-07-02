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
#include "fastlio_odom/imu_process.hh"
#include "fastlio_odom/lidar_process.hh"
namespace slam::fastlio {

class FastLioOdom {
   public:
    FastLioOdom();

   private:
    std::shared_ptr<IMUProcess> imu_process_ptr_;
    std::shared_ptr<FastLidarProcess> lidar_process_ptr_;
};
}  // namespace slam::fastlio