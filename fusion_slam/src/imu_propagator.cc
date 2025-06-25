#include "imu_propagator.hh"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <cstdint>
#include <memory>
#include "common/logger.hpp"
#include "lio-ieskf/ieskf.hh"
#include "static_imu_init.hh"

namespace slam {

ImuProgator::ImuProgator() {
    static_imu_init_ = std::make_shared<StateicImuInit>();
    ieskf_ptr_ = std::make_shared<IESKF>();
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
        // 这里还要对gnss对齐，一般是有一段gnss轨迹之后才开始对齐
    }
    // imu的中值积分递推
    Propagation(measures, state);
}

void ImuProgator::Propagation(const MeasureGroup& measures, NaviState& state) {
    // copy imus
    auto v_imus = measures.imus;
    v_imus.push_front(last_imu_data);
    const uint64_t& imu_begin_time = v_imus.front().timestamped_;
    const uint64_t& imu_end_time = v_imus.back().timestamped_;
    const double& pcl_beg_time = measures.lidar_begin_time;
    const double& pcl_end_time = measures.lidar_end_time;
    // 中值积分
    // 定义IMUPoses，用来给lidar去畸变
}

}  // namespace slam