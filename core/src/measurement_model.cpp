#include "mcl/measurement_model.hpp"

#include <cmath>

namespace mcl {

MeasurementModel::MeasurementModel(const MCLConfig& config)
    : likelihood_hit_(config.measurement_model.likelihood_hit),
      likelihood_miss_(config.measurement_model.likelihood_miss) {}

void MeasurementModel::SetMap(const std::vector<int8_t>& map_data,
                              const Vector2Type& origin, double resolution,
                              int width, int height) {
  // Store map data
  map_data_ = map_data;
  
  // Store map parameters
  map_origin_ = origin;
  map_resolution_ = resolution;
  map_width_ = width;
  map_height_ = height;
}

double MeasurementModel::ComputeLikelihood(const SE2Type& particle_pose,
                                           const std::vector<double>& ranges,
                                           double angle_min,
                                           double angle_increment,
                                           double range_min, double range_max,
                                           int beam_skip) const {
  double likelihood = 1.0;

  // For each laser beam
  for (size_t i = 0; i < ranges.size(); i += beam_skip) {
    double angle = angle_min + i * angle_increment;
    double measured_range = ranges[i];

    // Skip invalid measurements
    if (!std::isfinite(measured_range) || measured_range < range_min ||
        measured_range > range_max) {
      continue;
    }

    // Calculate beam endpoint in particle's frame
    double beam_x = measured_range * std::cos(angle);
    double beam_y = measured_range * std::sin(angle);

    // Transform to world coordinates
    Vector2Type beam_point = particle_pose * Vector2Type(beam_x, beam_y);

    // Convert to map cell coordinates
    int cell_x, cell_y;
    if (WorldToMap(beam_point, cell_x, cell_y)) {
      int idx = cell_y * map_width_ + cell_x;
      int occupancy = map_data_[idx];

      if (occupancy >= 0) {           // Known cell
        double p = likelihood_miss_;  // base probability
        if (occupancy > 50) {         // Occupied cell
          p = likelihood_hit_;
        }
        likelihood *= p;
      }
    }
  }

  return likelihood;
}

bool MeasurementModel::WorldToMap(const Vector2Type& world_point, int& cell_x,
                                  int& cell_y) const {
  Vector2Type p = (world_point - map_origin_) / map_resolution_;
  cell_x = static_cast<int>(p.x());
  cell_y = static_cast<int>(p.y());

  return cell_x >= 0 && cell_x < map_width_ && cell_y >= 0 &&
         cell_y < map_height_;
}

}  // namespace mcl