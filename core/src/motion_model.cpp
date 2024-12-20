#include "mcl/motion_model.hpp"

namespace mcl {

MotionModel::MotionModel(const MCLConfig& config)
    : noise_x_(config.motion_model.noise_x),
      noise_y_(config.motion_model.noise_y),
      noise_theta_(config.motion_model.noise_theta) {}

SE2Type MotionModel::Sample(const SE2Type& current_pose,
                           const SE2Type& odometry_delta,
                           std::mt19937& gen) const {
  // Create noise distributions
  std::normal_distribution<double> noise_x(0, noise_x_);
  std::normal_distribution<double> noise_y(0, noise_y_);
  std::normal_distribution<double> noise_theta(0, noise_theta_);
  
  // Extract delta translation and rotation
  Vector2Type delta_trans = odometry_delta.translation();
  double delta_theta = odometry_delta.so2().log();
  
  // Add noise
  Vector2Type noisy_trans(
      delta_trans.x() + noise_x(gen),
      delta_trans.y() + noise_y(gen)
  );
  double noisy_theta = delta_theta + noise_theta(gen);
  
  // Create noisy transform
  SE2Type noisy_delta(Sophus::SO2d(noisy_theta), noisy_trans);
  
  // Apply noisy transform to current pose
  return current_pose * noisy_delta;
}

SE2Type MotionModel::AddSmallNoise(const SE2Type& pose, std::mt19937& gen) const {
  // Add small noise to prevent particle depletion
  std::normal_distribution<double> noise_x(0, noise_x_);
  std::normal_distribution<double> noise_y(0, noise_y_);
  std::normal_distribution<double> noise_theta(0, noise_theta_);

  Vector2Type noisy_trans(
      pose.translation().x() + noise_x(gen),
      pose.translation().y() + noise_y(gen)
  );
  double noisy_theta = pose.so2().log() + noise_theta(gen);

  return SE2Type(Sophus::SO2d(noisy_theta), noisy_trans);
}

}  // namespace mcl
