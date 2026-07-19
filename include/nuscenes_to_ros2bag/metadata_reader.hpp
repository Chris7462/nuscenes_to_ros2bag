#pragma once

// C++
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

// Third party
#include <nlohmann/json.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// local
#include "nuscenes_to_ros2bag/metadata_types.hpp"

namespace fs = std::filesystem;

class MetadataReader
{
public:
  explicit MetadataReader(const rclcpp::Logger & logger);

  // Loads scene.json, sample.json, sample_data.json, ego_pose.json,
  // calibrated_sensor.json, and sensor.json from metadata_path.
  void load_from_directory(const fs::path & metadata_path);

  std::optional<SceneInfo> get_scene_info_by_number(uint32_t scene_number) const;

  // Returns every sample_data entry (keyframe or sweep, any modality)
  // belonging to the given scene, in no particular order.
  std::vector<SampleDataInfo> get_scene_sample_data(const Token & scene_token) const;

  EgoPoseInfo get_ego_pose_info(const Token & ego_pose_token) const;
  CalibratedSensorInfo get_calibrated_sensor_info(const Token & calibrated_sensor_token) const;
  CalibratedSensorName get_sensor_name(const Token & sensor_token) const;

private:
  static nlohmann::json slurp_json_file(const fs::path & file_path);

  void load_scenes(const fs::path & file_path);
  void load_samples(const fs::path & file_path);
  void load_sample_data(const fs::path & file_path);
  void load_ego_pose(const fs::path & file_path);
  void load_calibrated_sensor(const fs::path & file_path);
  void load_sensor(const fs::path & file_path);

  rclcpp::Logger logger_;

  std::vector<SceneInfo> scenes_;
  std::map<Token, Token> sample_token_to_scene_token_;
  std::vector<SampleDataInfo> sample_data_;
  std::map<Token, EgoPoseInfo> ego_pose_by_token_;
  std::map<Token, CalibratedSensorInfo> calibrated_sensor_by_token_;
  std::map<Token, CalibratedSensorName> sensor_name_by_token_;
};
