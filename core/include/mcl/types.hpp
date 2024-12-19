#ifndef MCL_TYPES_HPP_
#define MCL_TYPES_HPP_

#include <Eigen/Dense>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <memory>
#include <vector>

namespace mcl {

// Forward declarations
class ParticleFilter;
class MotionModel;
class MeasurementModel;

// Type definitions for 2D
using SE2Type = Sophus::SE2d;
using Vector2Type = Eigen::Vector2d;
using Matrix2Type = Eigen::Matrix2d;

// Type definitions for future 3D support
using SE3Type = Sophus::SE3d;
using Vector3Type = Eigen::Vector3d;
using Matrix3Type = Eigen::Matrix3d;

struct Particle {
    SE2Type pose;  // Will be replaced with SE3Type for 3D
    double weight;
    
    Particle() : weight(0.0) {}
    Particle(const SE2Type& p, double w) : pose(p), weight(w) {}
};

using Particles = std::vector<Particle>;
using ParticleFilterPtr = std::shared_ptr<ParticleFilter>;
using MotionModelPtr = std::shared_ptr<MotionModel>;
using MeasurementModelPtr = std::shared_ptr<MeasurementModel>;

}  // namespace mcl

#endif  // MCL_TYPES_HPP_ 