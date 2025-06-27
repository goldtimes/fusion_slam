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
    virtual ~OdomMatcher();

   protected:
    PointCloudPtr current_cloud_;
    bool first_scan = true;
    std::shared_ptr<IESKF> ieskf_ptr;
    SE3 last_pose_;
};
}  // namespace slam