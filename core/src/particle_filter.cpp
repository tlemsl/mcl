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
      // std::cout << "current_idx: " << current_idx << std::endl;
      new_particle.pose = particles_[current_idx].pose;
      new_particle.weight = 1.0/num_particles_;
      new_particles.push_back(std::move(new_particle));
  }
  
  for (auto& particle : new_particles) {
    // Add small noise to prevent particle depletion
    particle.pose = motion_model_->AddSmallNoise(particle.pose, gen_);
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
