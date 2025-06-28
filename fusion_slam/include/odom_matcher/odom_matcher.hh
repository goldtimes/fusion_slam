/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-27 21:50:04
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-28 13:31:21
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/odom_matcher/odom_matcher.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <memory>
#include "common/eigen_type.hh"
#include "lio-ieskf/ieskf.hh"
#include "sensors/lidar.hh"

namespace slam {
class OdomMatcher {
   public:
    virtual void Align() = 0;
    virtual void AddCloud(const PointCloudPtr& cloud) = 0;
    virtual SE3 GetCurrentPose() = 0;
    virtual ~OdomMatcher();

   protected:
    PointCloudPtr current_cloud_;
    bool first_scan = true;
    std::shared_ptr<IESKF> ieskf_ptr;
    SE3 last_pose_;
};
}  // namespace slam