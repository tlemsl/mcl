#ifndef MCL_CONFIG_HPP_
#define MCL_CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <string>

struct MotionModelConfig {
  double noise_x;
  double noise_y;
  double noise_theta;
};
struct MeasurementModelConfig {
  int beam_skip;
  double likelihood_hit;
  double likelihood_miss;
};

namespace mcl {
struct MCLConfig {
  int num_particles;
  double lambda;
  MotionModelConfig motion_model;
  MeasurementModelConfig measurement_model;
};

struct ROSConfig {
  std::string base_frame;
  std::string odom_frame;
  std::string map_frame;
  std::string scan_topic;
  std::string map_topic;
  std::string odom_topic;
  std::string pose_topic;
  std::string particles_topic;
  std::string initial_pose_topic;
};

struct FullConfig {
  MCLConfig mcl_config;
  ROSConfig ros_config;
};

class ConfigLoader {
 public:
  static FullConfig LoadFromYaml(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    FullConfig full_config;
    full_config.mcl_config = LoadMCLConfig(config);
    full_config.ros_config = LoadROSConfig(config);
    return full_config;
  }

 private:
  static MCLConfig LoadMCLConfig(const YAML::Node& config) {
    MCLConfig mcl_config;
    mcl_config.num_particles =
        config["particle_filter"]["num_particles"].as<int>(1000);
    mcl_config.lambda =
        config["particle_filter"]["lambda"].as<double>(1.0);
    mcl_config.motion_model.noise_x =
        config["motion_model"]["noise"]["x"].as<double>(0.02);
    mcl_config.motion_model.noise_y =
        config["motion_model"]["noise"]["y"].as<double>(0.02);
    mcl_config.motion_model.noise_theta =
        config["motion_model"]["noise"]["theta"].as<double>(0.01);
    mcl_config.measurement_model.beam_skip =
        config["measurement_model"]["beam_skip"].as<int>(10);
    mcl_config.measurement_model.likelihood_hit =
        config["measurement_model"]["likelihood"]["hit"].as<double>(0.9);
    mcl_config.measurement_model.likelihood_miss =
        config["measurement_model"]["likelihood"]["miss"].as<double>(0.1);
    return mcl_config;
  }
  static ROSConfig LoadROSConfig(const YAML::Node& config) {
    ROSConfig ros_config;
    ros_config.base_frame =
        config["ros_wrapper"]["frames"]["base"].as<std::string>("base_link");
    ros_config.odom_frame =
        config["ros_wrapper"]["frames"]["odom"].as<std::string>("odom");
    ros_config.map_frame =
        config["ros_wrapper"]["frames"]["map"].as<std::string>("map");
    ros_config.scan_topic =
        config["ros_wrapper"]["topics"]["scan"].as<std::string>("scan");
    ros_config.map_topic =
        config["ros_wrapper"]["topics"]["map"].as<std::string>("map");
    ros_config.odom_topic =
        config["ros_wrapper"]["topics"]["odom"].as<std::string>("odom");
    ros_config.pose_topic =
        config["ros_wrapper"]["topics"]["pose"].as<std::string>("pose");
    ros_config.particles_topic =
        config["ros_wrapper"]["topics"]["particles"].as<std::string>("particles");
    ros_config.initial_pose_topic =
        config["ros_wrapper"]["topics"]["initial_pose"].as<std::string>("initial_pose");
    return ros_config;
  }
};

}  // namespace mcl

#endif  // MCL_CONFIG_HPP_