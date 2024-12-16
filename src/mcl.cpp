#include "mcl/mcl.h"
#include <tf2/convert.h>
#include <tf2/utils.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

MCL::MCL(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : 
    nh_(nh),
    private_nh_(private_nh),
    tf_listener_(tf_buffer_),
    gen_(rd_()),
    first_odom_(true) {
    
    // Load parameters from private namespace
    private_nh_.param("num_particles", num_particles_, 1000);
    private_nh_.param("motion_noise/x", motion_noise_x_, 0.02);
    private_nh_.param("motion_noise/y", motion_noise_y_, 0.02);
    private_nh_.param("motion_noise/theta", motion_noise_theta_, 0.01);
    private_nh_.param("beam_skip", beam_skip_, 10);
    private_nh_.param("likelihood/hit", likelihood_hit_, 0.9);
    private_nh_.param("likelihood/miss", likelihood_miss_, 0.1);
    
    // Setup ROS communications
    scan_sub_ = nh_.subscribe("/scan", 1, &MCL::scanCallback, this);
    map_sub_ = nh_.subscribe("/map", 1, &MCL::mapCallback, this);
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1, &MCL::odomCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("mcl_pose", 1);
    particle_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_poses", 1);
    initial_pose_sub_ = nh_.subscribe("initialpose", 1, &MCL::initialPoseCallback, this);
    
    ROS_INFO("MCL initialized with %d particles", num_particles_);
}

void MCL::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_ = msg;
    if (particles_.empty()) {
        initializeParticles();
    }
}

void MCL::initializeParticles() {
    if (!map_) {
        ROS_ERROR("Cannot initialize particles without a map");
        return;
    }

    particles_.clear();
    particles_.reserve(num_particles_);
    
    std::uniform_real_distribution<double> dist_x(0, map_->info.width * map_->info.resolution);
    std::uniform_real_distribution<double> dist_y(0, map_->info.height * map_->info.resolution);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);
    
    for (int i = 0; i < num_particles_; ++i) {
        // Create a new particle
        Particle particle;
        
        // Generate random position and orientation
        double x = dist_x(gen_);
        double y = dist_y(gen_);
        double theta = dist_theta(gen_);
        
        // Initialize SE2 pose
        particle.pose = Sophus::SE2d(theta, Eigen::Vector2d(x, y));
        particle.weight = 1.0 / num_particles_;
        
        particles_.push_back(std::move(particle));
    }
    
    ROS_INFO("Initialized %zu particles", particles_.size());
}

void MCL::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (!map_ || particles_.empty()) return;
    
    // Apply motion to particles with noise
    Sophus::SE2d odom_delta = last_odom_.inverse() * current_odom_;
    // ROS_INFO("odom_delta: %f, %f, %f", odom_delta.translation().x(), odom_delta.translation().y(), odom_delta.so2().log());
    updateParticlesWithMotion(odom_delta);
    // Perform measurement update
    measurementUpdate(msg);
    
    // Resample if needed (you might want to do this less frequently)
    static int scan_count = 0;
    if (++scan_count >= 2) {  // Resample every 10 scans
        resample();
        scan_count = 0;
    }
    
    // Publish results
    publishResults();

    last_odom_ = current_odom_;
}

double MCL::measurementModel(const Particle& particle, const sensor_msgs::LaserScan::ConstPtr& scan) {
    double likelihood = 1.0;
    
    // Get scan parameters
    double angle_min = scan->angle_min;
    double angle_increment = scan->angle_increment;
    
    // For each laser beam
    for (size_t i = 0; i < scan->ranges.size(); i += beam_skip_) { // Skip some beams for efficiency
        double angle = angle_min + i * angle_increment;
        double measured_range = scan->ranges[i];
        
        // Skip invalid measurements
        if (!std::isfinite(measured_range) || 
            measured_range < scan->range_min || 
            measured_range > scan->range_max) {
            continue;
        }
        
        // Calculate expected beam endpoint in particle's frame
        double beam_x = measured_range * cos(angle);
        double beam_y = measured_range * sin(angle);
        
        // Transform to map coordinates
        Eigen::Vector2d beam_point = particle.pose * Eigen::Vector2d(beam_x, beam_y);
        
        // Convert to map cell coordinates
        int cell_x = (beam_point.x() - map_->info.origin.position.x) / map_->info.resolution;
        int cell_y = (beam_point.y() - map_->info.origin.position.y) / map_->info.resolution;
        
        // Check if point is within map bounds
        if (cell_x >= 0 && static_cast<unsigned int>(cell_x) < map_->info.width && 
            cell_y >= 0 && static_cast<unsigned int>(cell_y) < map_->info.height) {
            
            // Get occupancy value (-1: unknown, 0: free, 100: occupied)
            int idx = cell_y * map_->info.width + cell_x;
            int occupancy = map_->data[idx];
            
            if (occupancy >= 0) {  // Known cell
                // Calculate likelihood based on occupancy
                double p = likelihood_miss_;  // base probability
                if (occupancy > 50) {  // Occupied cell
                    p = likelihood_hit_;  // high probability for hits matching obstacles
                }
                likelihood *= p;
            }
        }
    }
    
    return likelihood;
}

void MCL::measurementUpdate(const sensor_msgs::LaserScan::ConstPtr& scan) {
    double weight_sum = 0;
    for (auto& particle : particles_) {
        particle.weight *= measurementModel(particle, scan);
        weight_sum += particle.weight;
    }
    
    // Normalize weights
    for (auto& particle : particles_) {
        particle.weight /= weight_sum;
    }
}

void MCL::resample() {
    if (particles_.empty()) {
        ROS_WARN("No particles to resample");
        return;
    }
    
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);  // Pre-allocate memory
    
    // Calculate cumulative weights
    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    
    for (int i = 1; i < num_particles_; i++) {
        cumulative_weights[i] = cumulative_weights[i-1] + particles_[i].weight;
    }
    
    // Normalize cumulative weights
    double total_weight = cumulative_weights.back();
    if (total_weight <= 0) {
        ROS_WARN("Total particle weight is zero, skipping resampling");
        return;
    }
    
    // Low variance resampler
    std::uniform_real_distribution<double> dist(0.0, 1.0/num_particles_);
    double r = dist(gen_);
    int current_idx = 0;
    
    for (int i = 0; i < num_particles_; i++) {
        double u = r + i * (1.0/num_particles_);
        
        while (u > (cumulative_weights[current_idx] / total_weight) && 
               current_idx < num_particles_ - 1) {
            current_idx++;
        }
        
        Particle new_particle;
        new_particle.pose = particles_[current_idx].pose;
        new_particle.weight = 1.0/num_particles_;
        new_particles.push_back(std::move(new_particle));
    }
    
    particles_ = std::move(new_particles);
}

void MCL::publishResults() {
    if (particles_.empty()) {
        return;
    }

    // Publish particle poses
    geometry_msgs::PoseArray particle_msg;
    particle_msg.header.stamp = ros::Time::now();
    particle_msg.header.frame_id = "map";
    
    for (const auto& particle : particles_) {
        geometry_msgs::Pose pose;
        // Get translation
        Eigen::Vector2d t = particle.pose.translation();
        pose.position.x = t.x();
        pose.position.y = t.y();
        pose.position.z = 0.0;
        
        // Get rotation as quaternion
        double theta = particle.pose.so2().log();
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        
        particle_msg.poses.push_back(pose);
    }
    particle_pub_.publish(particle_msg);

    // Compute weighted mean pose
    Eigen::Vector2d mean_translation = Eigen::Vector2d::Zero();
    double mean_theta = 0.0;
    double total_weight = 0.0;

    for (const auto& particle : particles_) {
        mean_translation += particle.weight * particle.pose.translation();
        mean_theta += particle.weight * particle.pose.so2().log();
        total_weight += particle.weight;
    }

    if (total_weight > 0) {
        mean_translation /= total_weight;
        mean_theta /= total_weight;
    }

    // Create SE2 from mean pose
    Eigen::Matrix2d R = Sophus::SO2d(mean_theta).matrix();
    Eigen::Vector2d t = mean_translation;
    Sophus::SE2d mean_pose(R, t);

    // Create and publish pose message
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    
    pose_msg.pose.pose.position.x = mean_translation.x();
    pose_msg.pose.pose.position.y = mean_translation.y();
    pose_msg.pose.pose.position.z = 0.0;

    double theta = mean_pose.so2().log();
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose_msg.pose.pose.orientation.w = q.w();
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    
    pose_pub_.publish(pose_msg);

    // Publish transform
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";

    // First convert our mean pose (map to base_link) to SE2d
    Sophus::SE2d T_map_base(mean_pose);

    // Get the inverse of current_odom_ (which is base_link to odom)
    Sophus::SE2d T_base_odom = current_odom_;
    Sophus::SE2d T_odom_base = T_base_odom.inverse();

    // Calculate the map to odom transform
    Sophus::SE2d T_map_odom = T_map_base * T_odom_base;

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

void MCL::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {   
    // Extract position
    current_odom_.translation() = Eigen::Vector2d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y
    );
    
    // Extract rotation
    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    current_odom_.so2() = Sophus::SO2d(yaw);
    
    // Create SE3 from the transform matrix
    
    if (first_odom_) {
        last_odom_ = current_odom_;
        first_odom_ = false;
        return;
    }
}

void MCL::updateParticlesWithMotion(const Sophus::SE2d& odom_delta) {
    std::normal_distribution<double> noise_x(0, motion_noise_x_);
    std::normal_distribution<double> noise_y(0, motion_noise_y_);
    std::normal_distribution<double> noise_theta(0, motion_noise_theta_);
    
    for (auto& particle : particles_) {
        // Extract delta translation and rotation
        Eigen::Vector2d delta_trans = odom_delta.translation();
        double delta_theta = odom_delta.so2().log();
        
        // Add noise
        Eigen::Vector2d noisy_trans(
            delta_trans.x() + noise_x(gen_),
            delta_trans.y() + noise_y(gen_)
        );
        double noisy_theta = delta_theta + noise_theta(gen_);
        
        // Create noisy transform
        Sophus::SE2d noisy_transform(Sophus::SO2d(noisy_theta), noisy_trans);
        
        // Update particle pose
        particle.pose = particle.pose * noisy_transform;
    }
} 

void MCL::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!map_) {
        ROS_WARN("Cannot set initial pose without a map");
        return;
    }

    // Clear existing particles
    particles_.clear();
    particles_.reserve(num_particles_);

    // Extract pose from message
    Eigen::Vector2d initial_trans(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y
    );

    // Convert quaternion to yaw angle
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double initial_yaw = tf2::getYaw(q);

    // Get standard deviations from covariance matrix
    double std_x = std::sqrt(msg->pose.covariance[0]);  // x variance
    double std_y = std::sqrt(msg->pose.covariance[7]);  // y variance
    double std_theta = std::sqrt(msg->pose.covariance[35]);  // yaw variance

    // Create normal distributions
    std::normal_distribution<double> dist_x(initial_trans.x(), std_x);
    std::normal_distribution<double> dist_y(initial_trans.y(), std_y);
    std::normal_distribution<double> dist_theta(initial_yaw, std_theta);

    // Generate particles around initial pose
    for (int i = 0; i < num_particles_; ++i) {
        Particle particle;
        
        // Sample pose with noise
        double x = dist_x(gen_);
        double y = dist_y(gen_);
        double theta = dist_theta(gen_);

        // Create SE2 transformation
        particle.pose = Sophus::SE2d(theta, Eigen::Vector2d(x, y));
        particle.weight = 1.0 / num_particles_;
        
        particles_.push_back(std::move(particle));
    }

    ROS_INFO("Initialized %zu particles around pose (x: %.2f, y: %.2f, yaw: %.2f)",
             particles_.size(), initial_trans.x(), initial_trans.y(), initial_yaw);
} 