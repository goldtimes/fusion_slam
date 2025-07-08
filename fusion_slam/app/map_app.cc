#include <ros/ros.h>
#include <iostream>
#include "common/logger.hh"
#include "map_node_build.hh"

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_build_node");
    ros::NodeHandle nh_;
    std::shared_ptr<slam::MapBuildNode> map_node_ptr = std::make_shared<slam::MapBuildNode>(nh_);
    map_node_ptr->Run();
    LOG_INFO("System exited");
    return 0;
}