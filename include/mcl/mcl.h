#ifndef MCL_MCL_H
#define MCL_MCL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <sophus/se2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <random>

struct Particle {
    Sophus::SE2d pose;
    double weight;
    
    Particle() : weight(0.0) {}
    
    Particle(const Particle& other) : 
        pose(other.pose),
        weight(other.weight) {}
        
    Particle& operator=(const Particle& other) {
        if (this != &other) {
            pose = other.pose;
            weight = other.weight;
        }
        return *this;
    }
};

class MCL {
public:
    MCL(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~MCL() = default;

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void initializeParticles();
    void measurementUpdate(const sensor_msgs::LaserScan::ConstPtr& scan);
    void resample();
    void publishResults();
    double measurementModel(const Particle& particle, const sensor_msgs::LaserScan::ConstPtr& scan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void updateParticlesWithMotion(const Sophus::SE2d& odom_delta);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
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
    
    // MCL parameters
    std::vector<Particle> particles_;
    nav_msgs::OccupancyGrid::ConstPtr map_;
    int num_particles_;
    
    // Motion model parameters
    double motion_noise_x_;
    double motion_noise_y_;
    double motion_noise_theta_;
    
    // Measurement model parameters
    int beam_skip_;
    double likelihood_hit_;
    double likelihood_miss_;
    
    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    
    // Latest odometry transform
    Sophus::SE2d last_odom_;
    Sophus::SE2d current_odom_;
    bool first_odom_;
}; 

#endif // MCL_MCL_H