#ifndef MCL_PARTICLE_FILTER_HPP_
#define MCL_PARTICLE_FILTER_HPP_

#include "mcl/types.hpp"
#include "mcl/config.hpp"
#include "mcl/motion_model.hpp"
#include "mcl/measurement_model.hpp"
#include <random>

namespace mcl {

class ParticleFilter {
 public:
  explicit ParticleFilter(const MCLConfig& config);
  
  // Initialize particles uniformly in the given area
  void InitializeUniform(double x_min, double x_max,
                        double y_min, double y_max,
                        double theta_min, double theta_max);
  
  // Initialize particles with Gaussian distribution
  void InitializeGaussian(const SE2Type& mean, 
                         double std_x, double std_y, double std_theta);
  
  // Update particles with motion model
  void Predict(const SE2Type& delta_pose);
  
  // Update particle weights based on measurement
  void Update(const std::vector<double>& ranges,
             double angle_min,
             double angle_increment,
             double range_min,
             double range_max);
  void SetMap(const std::vector<int8_t>& map_data,
              const Vector2Type& origin, double resolution,
              int width, int height){
    measurement_model_->SetMap(map_data, origin, resolution, width, height);
  }
  // Resample particles
  void Resample();
  
  // Get current pose estimate
  SE2Type GetEstimatedPose() const;
  // Getters
  const Particles& particles() const { return particles_; }
  
 private:
  // Random number generation
  std::random_device rd_;
  std::mt19937 gen_;
  
  // Core components
  MotionModelPtr motion_model_;
  MeasurementModelPtr measurement_model_;
  
  // Particle set
  Particles particles_;
  
  // Configuration
  int num_particles_;
  int beam_skip_;
  double lambda_;
};

}  // namespace mcl

#endif  // MCL_PARTICLE_FILTER_HPP_ 