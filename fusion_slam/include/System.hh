/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-14 02:39:34
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-14 08:27:56
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/Sytem.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%
 */
#pragma once

#include <yaml-cpp/yaml.h>
#include <memory>
#include "ros/ros.h"

namespace slam {
class SystemConfig;
};

namespace slam {
class System {
   public:
    System(const ros::NodeHandle& nh);
    ~System() = default;

   private:
    void InitConfig();

   public:
   private:
    ros::NodeHandle nh_;
    // 不需要定义类的对象，因为是单例，程序的生命周期都存在
    // SystemConfig& sys_config;
};

}  // namespace slam
