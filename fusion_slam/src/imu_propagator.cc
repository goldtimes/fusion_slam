#include "imu_propagator.hh"
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <cstdint>
#include <memory>
#include "common/PoseTrans.hpp"
#include "common/eigen_type.hh"
#include "common/lidar_point_type.hh"
#include "common/logger.hpp"
#include "common/navi_state.hh"
#include "common/pointcloud_utils.hh"
#include "lio-ieskf/ieskf.hh"
#include "sensors/lidar.hh"
#include "static_imu_init.hh"

namespace slam {

ImuProgator::ImuProgator(const std::shared_ptr<IESKF>& ieskf) {
    static_imu_init_ = std::make_shared<StateicImuInit>();
    ieskf_ptr_ = ieskf;
}
ImuProgator::~ImuProgator() {
}

void ImuProgator::Process(const MeasureGroup& measures, NaviState& state, PointCloudPtr& pcl_out) {
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
        // 设置ieskf的状态
        IESKF::Options ieskf_options;
        ieskf_options.gyro_var_ = std::sqrt(static_imu_init_->GetCovGyro()[0]);
        ieskf_options.acce_var_ = std::sqrt(static_imu_init_->GetCovAcc()[0]);
        ieskf_ptr_->SetInitialConditions(ieskf_options, static_imu_init_->GetinitBg(), static_imu_init_->GetInitBa(),
                                         static_imu_init_->GetGravity());
        last_imu_data = static_imu_init_->last_imu_data;
        boost::posix_time::ptime init_end_time = boost::posix_time::microsec_clock::local_time();
        LOG_INFO("[Imu init time used] {:03.6f} ms", (init_end_time - init_start_time).total_microseconds() * 1e-6);
        // 这里还要对gnss对齐，一般是有一段gnss轨迹之后才开始对齐
        return;
    }
    // imu的中值积分递推
    Propagation(measures, state, pcl_out);
}

void ImuProgator::Propagation(const MeasureGroup& measures, NaviState& state, PointCloudPtr& pcl_out) {
    // copy imus
    auto v_imus = measures.imus;
    v_imus.push_front(last_imu_data);
    const double& imu_begin_time = v_imus.front().timestamped_;
    const double& imu_end_time = v_imus.back().timestamped_;

    const double& pcl_beg_time = measures.lidar_begin_time;
    const double& pcl_end_time = measures.lidar_end_time;
    LOG_INFO("imu_begin_time:{},imu_end_time:{}", imu_begin_time, imu_end_time);
    LOG_INFO("pcl_beg_time:{},pcl_end_time:{}", pcl_beg_time, pcl_end_time);

    double dt;
    Eigen::Vector3d acc_avr;
    Eigen::Vector3d gyro_avr;
    // 中值积分
    imu_states_.clear();
    NaviState imu_begin_state(0.0, ieskf_ptr_->GetNominalSE3().so3(), ieskf_ptr_->GetNominalSE3().translation(),
                              ieskf_ptr_->GetNominalState().v_, ieskf_ptr_->GetNominalState().bg_,
                              ieskf_ptr_->GetNominalState().ba_);
    imu_states_.emplace_back(imu_begin_state);
    imu_states_.back().acc_ = Vec3d::Zero();
    imu_states_.back().gyro_ = Vec3d::Zero();
    std::sort(pcl_out->begin(), pcl_out->end(),
              [](const PointXYZIRT& point_a, const PointXYZIRT& point_b) { return point_a.time < point_b.time; });
    auto imu_propagation_start = boost::posix_time::microsec_clock::local_time();
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
            dt = (tail.timestamped_ - last_lidar_end_time);
            if (dt > 0.1) {
                LOG_INFO("dt:{}", dt);
            }
        } else {
            dt = (tail.timestamped_ - head.timestamped_);
            if (dt > 0.1) {
                LOG_INFO("dt:{}", dt);
            }
        }
        // 预测imu的位姿
        ieskf_ptr_->Predict(dt, acc_avr, gyro_avr);
        // odom更新和gnss更新
        const double offset_time = (tail.timestamped_ - pcl_beg_time);
        NaviState imu_state(offset_time, ieskf_ptr_->GetNominalSE3().so3(), ieskf_ptr_->GetNominalSE3().translation(),
                            ieskf_ptr_->GetNominalState().v_, ieskf_ptr_->GetNominalState().bg_,
                            ieskf_ptr_->GetNominalState().ba_);
        imu_state.acc_ = ieskf_ptr_->GetNominalSE3().so3().matrix() * (acc_avr - ieskf_ptr_->GetNominalState().ba_);
        imu_state.gyro_ = (gyro_avr - ieskf_ptr_->GetNominalState().bg_);
        imu_states_.emplace_back(imu_state);
        std::cout << "state: " << imu_states_.back() << std::endl;
    }
    //处理最后一个imu数据
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time) * 1e-6;
    ieskf_ptr_->Predict(dt, acc_avr, gyro_avr);
    auto imu_propagation_end = boost::posix_time::microsec_clock::local_time();
    LOG_INFO("imu propagation used time:{}", (imu_propagation_end - imu_propagation_start).total_microseconds() * 1e-6);

    auto imu_end_state = ieskf_ptr_->GetNominalState();
    last_lidar_end_time = pcl_end_time;
    // 定义IMUPoses，用来给lidar去畸变
    // sort lidar by time
    pcl_out->clear();
    pcl_out = measures.curr_cloud;
    auto undistort_start_time = boost::posix_time::microsec_clock::local_time();

    auto it_pcl = pcl_out->end() - 1;
    LOG_INFO("cloud size:{}", pcl_out->size());
    // SavePcd("/data/origin.pcd", pcl_out);
    // 雷达时刻i的点，转到imu坐标系下。
    // 在转到i时刻的世界坐标系
    // 转到end时刻的imu坐标系，最后转到雷达坐标系，达到去畸变的效果
    // 可以用插值的方式,fast-lio这里用了另一种方式
    SO3 R_imu;
    Eigen::Vector3d vel_imu, pos_imu;
    Eigen::Vector3d acc_imu, gyro_imu;
    // 这里去畸变好慢
    for (auto it_state = imu_states_.end() - 1; it_state != imu_states_.begin(); it_state--) {
        auto head = it_state - 1;
        auto tail = it_state;
        // LOG_INFO("head time:{},tail time:{}", head->timestamp_, tail->timestamp_);
        R_imu = head->R_;
        vel_imu = head->v_;
        pos_imu = head->p_;
        acc_imu = tail->acc_;
        gyro_imu = tail->gyro_;
        // for (int i = 0; i < pcl_out->size(); ++i) {
        //     LOG_INFO("point time:{}", pcl_out->points[i].time);
        // }
        // 遍历所有的雷达点
        // P_compensate = T_IL.inv() * T_endIinG.inv() * T_iIinG * T_IL * P_iInL
        for (; it_pcl->time > head->timestamp_; it_pcl--) {
            // 计算点到head的dt
            auto dt = it_pcl->time - head->timestamp_;
            // LOG_INFO("dt:{}", dt);
            // 计算该时刻的imu姿态
            SO3 R_i = R_imu * Sophus::SO3d::exp(gyro_imu * dt);
            Vec3d t_i = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
            PoseTranseD T_IiInG(R_i.matrix(), t_i);
            PoseTranseD T_IendInG(imu_end_state.GetSE3().so3().matrix(), imu_end_state.GetSE3().translation());
            // 雷达点在lidar坐标系
            Vec3d P_i_inL(it_pcl->x, it_pcl->y, it_pcl->z);
            // i雷达点在imu坐标系
            Vec3d P_i_InI = T_LtoI_ * P_i_inL;
            // i雷达在在世界坐标系
            Vec3d P_compensate = T_LtoI_.inverse() * T_IendInG.inverse() * T_IiInG * P_i_InI;
            it_pcl->x = P_compensate[0];
            it_pcl->y = P_compensate[1];
            it_pcl->z = P_compensate[2];
            if (it_pcl == pcl_out->begin()) {
                break;
            }
        }
    }
    // SavePcd("/data/undistord.pcd", pcl_out);
    auto undistort_end_time = boost::posix_time::microsec_clock::local_time();
    LOG_INFO("lidar undistort used time:{}", (undistort_end_time - undistort_start_time).total_microseconds() * 1e-6);
    state = imu_end_state;
}

}  // namespace slam