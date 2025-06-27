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

   private:
    std::shared_ptr<IncNDT> inc_ndt_ptr_;
};
}  // namespace slam