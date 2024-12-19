#ifndef MCL_MEASUREMENT_MODEL_HPP_
#define MCL_MEASUREMENT_MODEL_HPP_

#include "mcl/types.hpp"
#include "mcl/config.hpp"
#include <vector>

namespace mcl {

class MeasurementModel {
 public:
  explicit MeasurementModel(const MCLConfig& config);
  
  // Set the current occupancy grid map
  void SetMap(const std::vector<int8_t>& map_data,
              const Vector2Type& origin,
              double resolution,
              int width,
              int height);
              
  // Compute measurement likelihood for a particle
  double ComputeLikelihood(const SE2Type& particle_pose,
                          const std::vector<double>& ranges,
                          double angle_min,
                          double angle_increment,
                          double range_min,
                          double range_max,
                          int beam_skip) const;
                          
 private:
  // Convert world coordinates to map cell coordinates
  bool WorldToMap(const Vector2Type& world_point,
                 int& cell_x, int& cell_y) const;
                 
  // Map data
  std::vector<int8_t> map_data_;
  Vector2Type map_origin_;
  double map_resolution_;
  int map_width_;
  int map_height_;
  
  // Measurement model parameters
  double likelihood_hit_;
  double likelihood_miss_;
};

}  // namespace mcl

#endif  // MCL_MEASUREMENT_MODEL_HPP_ 