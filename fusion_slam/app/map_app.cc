/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-08 23:14:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-08 23:51:02
 * @FilePath: /fusion_slam_ws/src/fusion_slam/app/map_app.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include <iostream>
#include "common/logger.hh"
#include "map_node_build.hh"

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_build_node");
    ros::NodeHandle nh_;
    std::shared_ptr<slam::MapBuildNode> map_node_ptr = std::make_shared<slam::MapBuildNode>(nh_);
    SpdLogger logger;
    map_node_ptr->Run();
    LOG_INFO("System exited");
    return 0;
}