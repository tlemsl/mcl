#include "mcl/particle_filter.hpp"

#include <algorithm>

namespace mcl {

ParticleFilter::ParticleFilter(const MCLConfig& config)
    : gen_(rd_()),
      num_particles_(config.num_particles),
      beam_skip_(config.measurement_model.beam_skip) {
  motion_model_ = std::make_shared<MotionModel>(config);
  measurement_model_ = std::make_shared<MeasurementModel>(config);

  particles_.reserve(num_particles_);
}

void ParticleFilter::InitializeUniform(double x_min, double x_max, double y_min,
                                       double y_max, double theta_min,
                                       double theta_max) {
  particles_.clear();
  particles_.reserve(num_particles_);

  std::uniform_real_distribution<double> dist_x(x_min, x_max);
  std::uniform_real_distribution<double> dist_y(y_min, y_max);
  std::uniform_real_distribution<double> dist_theta(theta_min, theta_max);

  for (int i = 0; i < num_particles_; ++i) {
    double x = dist_x(gen_);
    double y = dist_y(gen_);
    double theta = dist_theta(gen_);

    particles_.emplace_back(SE2Type(theta, Vector2Type(x, y)),
                            1.0 / num_particles_);
  }
}

void ParticleFilter::InitializeGaussian(const SE2Type& mean, double std_x,
                                        double std_y, double std_theta) {
  particles_.clear();
  particles_.reserve(num_particles_);
  std::normal_distribution<double> dist_x(mean.translation().x(), std_x);
  std::normal_distribution<double> dist_y(mean.translation().y(), std_y);
  std::normal_distribution<double> dist_theta(mean.so2().log(), std_theta);

  for (int i = 0; i < num_particles_; ++i) {
    double x = dist_x(gen_);
    double y = dist_y(gen_);
    double theta = dist_theta(gen_);

    particles_.emplace_back(SE2Type(theta, Vector2Type(x, y)),
                            1.0 / num_particles_);
  }
}

void ParticleFilter::Predict(const SE2Type& delta_pose) {
  for (auto& particle : particles_) {
    particle.pose = motion_model_->Sample(particle.pose, delta_pose, gen_);
  }
}

void ParticleFilter::Update(const std::vector<double>& ranges, double angle_min,
                            double angle_increment, double range_min,
                            double range_max) {
  double weight_sum = 0.0;

  for (auto& particle : particles_) {
    particle.weight *= measurement_model_->ComputeLikelihood(
        particle.pose, ranges, angle_min, angle_increment, range_min, range_max,
        beam_skip_);
    weight_sum += particle.weight;
  }

  // Normalize weights
  if (weight_sum > 0.0) {
    for (auto& particle : particles_) {
      particle.weight /= weight_sum;
    }
  } else {
    // If all weights are zero, reset to uniform weights
    double uniform_weight = 1.0 / particles_.size();
    for (auto& particle : particles_) {
      particle.weight = uniform_weight;
    }
  }
}

void ParticleFilter::Resample() {
  std::vector<double> cumsum(particles_.size());
  cumsum[0] = particles_[0].weight;

  for (size_t i = 1; i < particles_.size(); ++i) {
    cumsum[i] = cumsum[i - 1] + particles_[i].weight;
  }

  Particles new_particles;
  new_particles.reserve(num_particles_);

  std::uniform_int_distribution<int> dist(0, num_particles_ - 1);

  for (int i = 0; i < num_particles_; ++i) {
    new_particles.emplace_back(particles_[dist(gen_)].pose,
                               1.0 / num_particles_);
  }

  particles_ = std::move(new_particles);
}

SE2Type ParticleFilter::GetEstimatedPose() const {
  Vector2Type mean_translation = Vector2Type::Zero();
  double mean_theta = 0.0;
  double total_weight = 0.0;

  for (const auto& particle : particles_) {
    // double weight = exp(particle.weight / lambda_);
    double weight = particle.weight;
    mean_translation += weight * particle.pose.translation();
    mean_theta += weight * particle.pose.so2().log();
    total_weight += weight;
  }

  if (total_weight > 0.0) {
    mean_translation /= total_weight;
    mean_theta /= total_weight;
  }

  return SE2Type(mean_theta, mean_translation);
}

}  // namespace mcl
