#pragma once

// C++
#include <filesystem>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// local
#include "nuscenes_to_ros2bag/bag_writer.hpp"
#include "nuscenes_to_ros2bag/message_converter.hpp"
#include "nuscenes_to_ros2bag/metadata_reader.hpp"
#include "nuscenes_to_ros2bag/metadata_types.hpp"

namespace fs = std::filesystem;

class SceneConverter
{
public:
  SceneConverter(
    const MetadataReader & metadata_reader, MessageConverter & message_converter,
    BagWriter & bag_writer, const rclcpp::Logger & logger);

  // Converts every sample_data entry belonging to scene_info, in
  // chronological order, and writes each to the bag as it goes. This is an
  // eager, single-pass conversion (no timer/pacing) -- appropriate for an
  // offline batch tool with no real-time playback requirement.
  void run(const fs::path & dataroot, const SceneInfo & scene_info);

private:
  const MetadataReader & metadata_reader_;
  MessageConverter & message_converter_;
  BagWriter & bag_writer_;
  rclcpp::Logger logger_;
};
