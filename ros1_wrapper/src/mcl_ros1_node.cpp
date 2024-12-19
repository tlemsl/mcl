#include "mcl_ros/mcl_ros1_wrapper.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mcl_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  try {
    mcl_ros::MCLRos1Wrapper mcl(nh, private_nh);
    ROS_INFO("MCL node initialized successfully");
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_ERROR("MCL node failed: %s", e.what());
    return 1;
  }
  
  return 0;
} 