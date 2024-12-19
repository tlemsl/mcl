#include "mcl_ros/mcl_ros1_wrapper.hpp"

#include <tf2/utils.h>

namespace mcl_ros {

MCLRos1Wrapper::MCLRos1Wrapper(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      tf_listener_(tf_buffer_),
      first_odom_(true) {
  // Load config file path from parameter
  std::string config_path;
  private_nh_.param<std::string>("config_file", config_path,
                                 "config/mcl_params.yaml");

  // Load configuration
  YAML::Node config_node = YAML::LoadFile(config_path);

  // Initialize particle filter with config
  mcl::FullConfig full_config = mcl::ConfigLoader::LoadFromYaml(config_path);
  particle_filter_ =
      std::make_shared<mcl::ParticleFilter>(full_config.mcl_config);

  // Load ROS-specific parameters from config
  ros_config_ = full_config.ros_config;

  // Initialize ROS subscribers and publishers
  InitializeROSCommunication();
}

void MCLRos1Wrapper::InitializeROSCommunication() {
  scan_sub_ = nh_.subscribe(ros_config_.scan_topic, 1,
                           &MCLRos1Wrapper::ScanCallback, this);
  map_sub_ = nh_.subscribe(ros_config_.map_topic, 1,
                          &MCLRos1Wrapper::MapCallback, this);
  odom_sub_ = nh_.subscribe(ros_config_.odom_topic, 1,
                           &MCLRos1Wrapper::OdomCallback, this);
  initial_pose_sub_ = nh_.subscribe(ros_config_.initial_pose_topic, 1,
                                   &MCLRos1Wrapper::InitialPoseCallback, this);
  
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      ros_config_.pose_topic, 1);
  particle_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
      ros_config_.particles_topic, 1);
}

void MCLRos1Wrapper::InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  // Convert ROS pose to SE2
  mcl::Vector2Type initial_trans(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y);
  double initial_yaw = tf2::getYaw(msg->pose.pose.orientation);
  mcl::SE2Type initial_pose(initial_yaw, initial_trans);

  // Get standard deviations from covariance
  double std_x = std::sqrt(msg->pose.covariance[0]);
  double std_y = std::sqrt(msg->pose.covariance[7]);
  double std_theta = std::sqrt(msg->pose.covariance[35]);

  // Initialize particles with Gaussian distribution
  particle_filter_->InitializeGaussian(initial_pose, std_x, std_y, std_theta);
}

void MCLRos1Wrapper::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  map_ = msg;

  // Convert ROS map to MCL format
  std::vector<int8_t> map_data = msg->data;
  mcl::Vector2Type origin(msg->info.origin.position.x,
                          msg->info.origin.position.y);

  // Set map in measurement model
  particle_filter_->SetMap(map_data, origin, msg->info.resolution,
                           msg->info.width, msg->info.height);

  // Initialize particles if not initialized
  if (map_ && particle_filter_->particles().empty()) {
    double x_max = map_->info.width * map_->info.resolution;
    double y_max = map_->info.height * map_->info.resolution;
    particle_filter_->InitializeUniform(
        origin.x() - x_max / 2, origin.x() + x_max / 2, origin.y() - y_max / 2,
        origin.y() + y_max / 2, -M_PI, M_PI);
    ROS_INFO("Initialized particles uniformly in map bounds");
  }
}

void MCLRos1Wrapper::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (!map_ || particle_filter_->particles().empty()) {
    return;
  }

  // Update particles with motion model
  mcl::SE2Type odom_delta = last_odom_.inverse() * current_odom_;
  particle_filter_->Predict(odom_delta);

  // Update particle weights with measurement model
  std::vector<double> ranges_double(msg->ranges.begin(), msg->ranges.end());
  particle_filter_->Update(ranges_double, msg->angle_min, msg->angle_increment,
                           msg->range_min, msg->range_max);
  // Resample particles
  particle_filter_->Resample();

  // Update odometry
  last_odom_ = current_odom_;

  // Publish results
  PublishResults();
}

void MCLRos1Wrapper::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Convert ROS odometry to SE2
  mcl::Vector2Type trans(msg->pose.pose.position.x, msg->pose.pose.position.y);
  double yaw = tf2::getYaw(msg->pose.pose.orientation);
  current_odom_ = mcl::SE2Type(yaw, trans);

  if (first_odom_) {
    last_odom_ = current_odom_;
    first_odom_ = false;
  }
}

void MCLRos1Wrapper::PublishResults() {
  auto estimated_pose = particle_filter_->GetEstimatedPose();

  // Publish pose estimate
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = ros_config_.map_frame;

  // Set position
  pose_msg.pose.pose.position.x = estimated_pose.translation().x();
  pose_msg.pose.pose.position.y = estimated_pose.translation().y();
  pose_msg.pose.pose.position.z = 0.0;

  // Set orientation
  double theta = estimated_pose.so2().log();
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  pose_msg.pose.pose.orientation.w = q.w();
  pose_msg.pose.pose.orientation.x = q.x();
  pose_msg.pose.pose.orientation.y = q.y();
  pose_msg.pose.pose.orientation.z = q.z();

  pose_pub_.publish(pose_msg);

  // Publish particle cloud
  geometry_msgs::PoseArray particle_msg;
  particle_msg.header = pose_msg.header;
  auto particles = particle_filter_->particles();
  for (const auto& particle : particles) {
    geometry_msgs::Pose p;
    p.position.x = particle.pose.translation().x();
    p.position.y = particle.pose.translation().y();
    p.position.z = 0.0;

    double theta = particle.pose.so2().log();
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();

    particle_msg.poses.push_back(p);
  }
  particle_pub_.publish(particle_msg);

  // Broadcast transform
  geometry_msgs::TransformStamped transform;
  transform.header = pose_msg.header;
  transform.child_frame_id = ros_config_.odom_frame;

  // Calculate the map to odom transform
  mcl::SE2Type T_map_odom = estimated_pose * current_odom_.inverse();

  // Extract the translation and rotation
  Eigen::Vector2d t_map_odom = T_map_odom.translation();
  double theta_map_odom = T_map_odom.so2().log();

  // Fill in the transform message
  transform.transform.translation.x = t_map_odom.x();
  transform.transform.translation.y = t_map_odom.y();
  transform.transform.translation.z = 0.0;

  // Convert the rotation to quaternion
  q.setRPY(0, 0, theta_map_odom);
  transform.transform.rotation.w = q.w();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();

  tf_broadcaster_.sendTransform(transform);
}
}  // namespace mcl_ros