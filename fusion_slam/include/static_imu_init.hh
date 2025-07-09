#include "common/common_lib.hh"
#include "common/eigen_type.hh"
#include "common/logger.hh"
namespace slam {
class StaticImuInit {
   public:
    struct Options {
        Options() {
        }
        // 静止的时间
        double init_time_seconds = 1.0;
        // 队列长度
        int init_buffer_size = 1000;
        double max_static_gyro_var = 0.5;
        double max_static_acc_var = 0.05;
        double gravity_norm_ = 9.81;
        bool align_gravity = false;
    };
    explicit StaticImuInit(Options options = Options()) {
    }

    bool AddMeasurements(const MeasureGroup& measures);

    bool TryInit();

    void grad_schmit(const Eigen::Vector3d& gravity_inI, Eigen::Matrix3d& R_GtoI);

    bool GetInitSuccess() const {
        return init_success_;
    }

    const Eigen::Vector3d& GetCovGyro() const {
        return cov_gyro_;
    }
    const Eigen::Vector3d& GetCovAcc() const {
        return cov_acc_;
    }
    const Eigen::Vector3d& GetInitBa() const {
        return init_ba_;
    }
    const Eigen::Vector3d& GetinitBg() const {
        return init_bg_;
    }
    const Eigen::Vector3d& GetGravity() const {
        return gravity_;
    }
    const Eigen::Matrix3d& GetRGtoI() const {
        return R_GtoI;
    }

   public:
    IMUData last_imu_data;

   private:
    Options options_;
    std::deque<IMUData> init_imus_;
    bool init_success_ = false;
    bool try_to_init_ = false;
    // 静止的初始时间
    double init_start_time = 0.0;
    // 零偏
    Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero();
    // 测量噪声协方差对角线
    Eigen::Vector3d cov_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d cov_acc_ = Eigen::Vector3d::Zero();
    // 重力
    Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
    // 初始R_GtoI;
    Eigen::Matrix3d R_GtoI = Eigen::Matrix3d::Identity();
};
}  // namespace slam