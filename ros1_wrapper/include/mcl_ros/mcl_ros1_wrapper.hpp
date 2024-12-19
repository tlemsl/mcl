#ifndef MCL_ROS1_WRAPPER_HPP_
#define MCL_ROS1_WRAPPER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "mcl/particle_filter.hpp"
#include "mcl/config.hpp"

namespace mcl_ros {

class MCLRos1Wrapper {
 public:
  MCLRos1Wrapper(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  
 private:
  // Callbacks
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void InitializeROSCommunication();
  
  // Utility functions
  void PublishResults();
  
  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher particle_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  
  // MCL core
  std::shared_ptr<mcl::ParticleFilter> particle_filter_;
  nav_msgs::OccupancyGrid::ConstPtr map_;
    
  // Latest odometry
  mcl::SE2Type last_odom_;
  mcl::SE2Type current_odom_;
  bool first_odom_;

  // ROS config
  mcl::ROSConfig ros_config_;
};

}  // namespace mcl_ros

#endif  // MCL_ROS1_WRAPPER_HPP_ 