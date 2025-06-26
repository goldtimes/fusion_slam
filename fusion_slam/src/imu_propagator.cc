#include "imu_propagator.hh"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <cstdint>
#include <memory>
#include "common/logger.hpp"
#include "common/navi_state.hh"
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
    const uint64_t& pcl_beg_time = measures.lidar_begin_time;
    const uint64_t& pcl_end_time = measures.lidar_end_time;
    double dt;
    Eigen::Vector3d acc_avr;
    Eigen::Vector3d gyro_avr;
    // 中值积分
    imu_states_.clear();
    NaviState imu_begin_state = ieskf_ptr_->GetNominalState();
    imu_states_.emplace_back(imu_begin_state);
    // 遍历所有的imu数据
    for (auto it_imu = v_imus.begin(); it_imu < v_imus.end() - 1; it_imu++) {
        auto&& head = *it_imu;
        auto&& tail = *(it_imu + 1);
        acc_avr = (head.acc_ + tail.acc_) * 0.5;
        gyro_avr = (head.gyro_ + tail.gyro_) * 0.5;
        // 这里要注意dt的计算
        if (tail.timestamped_ < last_lidar_end_time) {
            LOG_INFO("imu begin time:{} <  last_lidar_end_time:{}", tail.timestamped_, last_lidar_end_time);
            continue;
        }
        if (head.timestamped_ < last_lidar_end_time) {
            dt = (last_lidar_end_time - head.timestamped_) * 1e-6;
            LOG_INFO("dt:{}", dt);
        } else {
            dt = (tail.timestamped_ - head.timestamped_) * 1e-6;
            LOG_INFO("dt:{}", dt);
        }
        // 预测imu的位姿
        ieskf_ptr_->Predict(dt, acc_avr, gyro_avr);
        // odom更新和gnss更新
        const double offset_time = (tail.timestamped_ - pcl_beg_time) * 1e-6;
        imu_states_.emplace_back(offset_time, ieskf_ptr_->GetNominalSE3().so3(),
                                 ieskf_ptr_->GetNominalSE3().translation(), ieskf_ptr_->GetNominalState().v_,
                                 ieskf_ptr_->GetNominalState().bg_, ieskf_ptr_->GetNominalState().ba_);
        std::cout << "state: " << imu_states_.back() << std::endl;
    }
    //处理最后一个imu数据
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time) * 1e-6;
    ieskf_ptr_->Predict(dt, acc_avr, gyro_avr);
    last_lidar_end_time = pcl_end_time;
    // NaviState imu_end_state();
    // 定义IMUPoses，用来给lidar去畸变
}

}  // namespace slam