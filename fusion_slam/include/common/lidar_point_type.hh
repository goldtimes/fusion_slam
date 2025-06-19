/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-15 11:21:43
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-20 00:28:53
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/common/lidar_point_type.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>  // 确保点类型完全定义
#define PCL_NO_PRECOMPILE

namespace slam {

struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring = 0;
    double timestamp = 0.0;  // unit: s

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct LsPointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY

    std::uint16_t ring = 0;
    double timestamp = 0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;  // uint: ns
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t noise;
    std::uint32_t range;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct LivoxMid360PointXYZITLT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY

    std::uint8_t tag;
    std::uint8_t line;
    double timestamp;  // 精确到纳秒(ns)的UTC时间戳
} EIGEN_ALIGN16;

struct LivoxPointXYZITLT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY
    std::uint32_t time;  // unit: ns
    std::uint8_t line;
    std::uint8_t tag;
} EIGEN_ALIGN16;

struct PointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    std::uint8_t ring;
    float time;  // offset time relative to the first point. unit: s

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace slam

POINT_CLOUD_REGISTER_POINT_STRUCT(slam::RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp))
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::LsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp))
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(std::uint16_t, ring,
                                                                                     ring)(float, time, time))
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                           reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, noise,
                                                                                                   noise)(std::uint32_t,
                                                                                                          range, range))
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::LivoxMid360PointXYZITLT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp, timestamp))
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::LivoxPointXYZITLT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint32_t, time, time)(std::uint8_t, line, line)(std::uint8_t, tag, tag))
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(std::uint8_t, ring,
                                                                                     ring)(float, time, time))