#ifndef MCL_MOTION_MODEL_HPP_
#define MCL_MOTION_MODEL_HPP_

#include "mcl/types.hpp"
#include "mcl/config.hpp"
#include <random>

namespace mcl {

class MotionModel {
 public:
  explicit MotionModel(const MCLConfig& config);
  
  // Sample a new pose given the current pose and odometry measurement
  SE2Type Sample(const SE2Type& current_pose, 
                 const SE2Type& odometry_delta,
                 std::mt19937& gen) const;
                 
 private:
  // Motion noise parameters
  double noise_x_;
  double noise_y_;
  double noise_theta_;
};

}  // namespace mcl

#endif  // MCL_MOTION_MODEL_HPP_ 
