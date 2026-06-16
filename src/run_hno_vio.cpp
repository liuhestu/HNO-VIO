#include "hno_vio/HNOManager.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "run_hno_vio");
    ros::NodeHandle nh("~");

    std::string config_path;
    nh.param<std::string>("config_path", config_path, "");

    auto manager = std::make_shared<hno_vio::HNOManager>(nh, config_path);
    manager->launch_subscribers();
    
    ROS_INFO("HNO Node Started (Merged Architecture).");
    ros::spin();
    return 0;
}
