/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-15 10:39:24
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-16 00:34:37
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/common/lidar_model.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <string>
#include "common/logger.hpp"
#include "common/math.hh"

namespace slam {
class LidarModel {
   public:
    enum class LIDAR_TYPE {
        AVIA = 0,
        MID360 = 1,
        LEISHEN16 = 2,
        ROBOSENSE16 = 3,
        Velodyne = 4,
        OUSTER128 = 5,
        NONE = 6,
    };
    LidarModel() = delete;
    LidarModel(const LidarModel &lidar_mode) = delete;
    LidarModel(LidarModel &&) = delete;
    LidarModel &operator=(const LidarModel &) = delete;

    ~LidarModel() {
        delete instance_;
    }
    static LidarModel *GetInstance(const std::string lidar_type = "");

   public:
    LIDAR_TYPE lidar_sensor_type_ = LIDAR_TYPE::NONE;
    int vertical_scan_num_{std::numeric_limits<int>::max()};  // number of scans
    int horizon_scan_num_{std::numeric_limits<int>::max()};   // number of horizontal scans
    float h_res_{std::numeric_limits<float>::max()};          // horizontal resolution, radian
    float v_res_{std::numeric_limits<float>::max()};          // vertical resolution, radian
    float lower_angle_{std::numeric_limits<float>::max()};    // lidar minimum detection angle(abs), radian

   private:
    explicit LidarModel(const std::string &lidar_type = "");

   private:
    static LidarModel *instance_;
    static std::string lidar_type_;
};
}  // namespace slam