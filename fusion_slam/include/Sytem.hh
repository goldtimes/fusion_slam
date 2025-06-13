/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-14 02:39:34
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-14 02:51:28
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/Sytem.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%
 */
#pragma  once

#include "ros/ros.h"
#include <memory>

namespace slam{
    class SystemConfig;
};

namespace slam {
    class System{
        public:
            System(const ros::NodeHandle& nh, const std::shared_ptr<SystemConfig>& config_ptr);
            ~System() = default;
        private:
            ros::NodeHandle nh_;
            std::shared_ptr<SystemConfig> sys_config_ptr_;

    };

}
