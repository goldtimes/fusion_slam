/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-27 21:50:04
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-28 13:31:31
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/odom_matcher/ndt_odom_matcher.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <pcl/common/transforms.h>
#include <memory>
#include "inc_ndt.hh"
#include "lio-ieskf/ieskf.hh"
#include "odom_matcher.hh"

namespace slam {
class NdtOdomMatcher : public OdomMatcher {
   public:
    NdtOdomMatcher(const std::shared_ptr<IESKF>& ieskf);
    ~NdtOdomMatcher();
    virtual void AddCloud(const PointCloudPtr& cloud) override;
    virtual void Align() override;
    virtual SE3 GetCurrentPose() override;

   private:
    std::shared_ptr<IncNDT> inc_ndt_ptr_;
};
}  // namespace slam