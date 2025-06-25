#include "imu_propagator.hh"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <memory>
#include "common/logger.hpp"
#include "static_imu_init.hh"

namespace slam {

ImuProgator::ImuProgator() {
    static_imu_init_ = std::make_shared<StateicImuInit>();
}
ImuProgator::~ImuProgator() {
}

void ImuProgator::Process(const MeasureGroup& measures, NaviState& state) {
    // 初始化imu
    if (!imu_inited_) {
        boost::posix_time::ptime init_start_time = boost::posix_time::microsec_clock::local_time();
        static_imu_init_->AddMeasurements(measures);
        static_imu_init_->TryInit();
        if (!static_imu_init_->GetInitSuccess()) {
            return;
        }
        // imu初始化成功
        imu_inited_ = true;
        last_imu_data = static_imu_init_->last_imu_data;
        boost::posix_time::ptime init_end_time = boost::posix_time::microsec_clock::local_time();
        LOG_INFO("[Imu init time used] {:03.6f} ms", (init_end_time - init_start_time).total_microseconds() * 1e-6);
    }
}

}  // namespace slam