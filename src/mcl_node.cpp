#include "mcl/mcl.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mcl_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");  // for private parameters
    
    try {
        // Create MCL instance
        MCL mcl(nh, private_nh);
        
        // Set up timer for periodic updates if needed
        double update_rate;
        private_nh.param("update_rate", update_rate, 20.0);  // default 20Hz
        ros::Rate rate(update_rate);
        
        ROS_INFO("MCL node initialized successfully");
        
        // Main loop
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    }
    catch (const std::exception& e) {
        ROS_ERROR("MCL node failed: %s", e.what());
        return 1;
    }
    
    return 0;
} 