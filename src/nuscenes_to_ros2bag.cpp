// C++
#include <filesystem>
#include <string>

// local
#include "nuscenes_to_ros2bag/nuscenes_to_ros2bag.hpp"
#include "nuscenes_to_ros2bag/bag_writer.hpp"
#include "nuscenes_to_ros2bag/message_converter.hpp"
#include "nuscenes_to_ros2bag/metadata_reader.hpp"
#include "nuscenes_to_ros2bag/scene_converter.hpp"

namespace fs = std::filesystem;


NuscenesToRos2bagNode::NuscenesToRos2bagNode()
: Node("nuscenes_to_ros2bag_node")
{
  const fs::path dataroot = declare_parameter("dataroot", fs::path());
  const std::string version = declare_parameter("version", std::string("v1.0-mini"));
  const int64_t scene_number = declare_parameter("scene_number", static_cast<int64_t>(-1));

  if (scene_number < 0) {
    RCLCPP_ERROR(get_logger(), "Parameter 'scene_number' is required and must be >= 0.");
    rclcpp::shutdown();
    return;
  }

  MetadataReader metadata_reader(get_logger());
  const fs::path metadata_path = dataroot / version;
  RCLCPP_INFO(get_logger(), "Loading metadata from %s ...", metadata_path.c_str());

  try {
    // If any metadata file is not found, a runtime_error is thrown.
    metadata_reader.load_from_directory(metadata_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  const auto scene_info_opt =
    metadata_reader.get_scene_info_by_number(static_cast<uint32_t>(scene_number));
  if (!scene_info_opt) {
    RCLCPP_ERROR(get_logger(), "Scene with ID=%ld not found!", scene_number);
    rclcpp::shutdown();
    return;
  }
  const auto & scene_info = scene_info_opt.value();

  const std::string output_bag_name = scene_info.name + "_bag";
  BagWriter bag_writer(output_bag_name);
  bag_writer.create_topics();

  MessageConverter message_converter(get_logger());

  SceneConverter scene_converter(metadata_reader, message_converter, bag_writer, get_logger());
  scene_converter.run(dataroot, scene_info);

  RCLCPP_INFO(get_logger(), "Wrote bag to '%s'.", output_bag_name.c_str());
}
