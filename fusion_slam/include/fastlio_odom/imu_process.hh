#pragma once
#include <memory>
#include "common/common_lib.hh"

namespace slam {
class IESKF;
class StaticImuInit;
}  // namespace slam
namespace slam {
class IMUProcessor {
   public:
    struct IMUProcessorConfig {};
    IMUProcessor(const IMUProcessorConfig& config, std::shared_ptr<IESKF> kf);

    void Process(MeasureGroup& sync_package);

   private:
    std::shared_ptr<IESKF> kf_;
    std::shared_ptr<StaticImuInit> static_imu_init_;
};
}  // namespace slam