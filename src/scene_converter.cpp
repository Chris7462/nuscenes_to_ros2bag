// C++
#include <algorithm>
#include <cctype>

// local
#include "nuscenes_to_ros2bag/scene_converter.hpp"

namespace
{

std::string to_lower(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return text;
}

}  // namespace


SceneConverter::SceneConverter(
  const MetadataReader & metadata_reader, MessageConverter & message_converter,
  BagWriter & bag_writer, const rclcpp::Logger & logger)
: metadata_reader_(metadata_reader),
  message_converter_(message_converter),
  bag_writer_(bag_writer),
  logger_(logger)
{
}

void SceneConverter::run(const fs::path & dataroot, const SceneInfo & scene_info)
{
  auto sample_datas = metadata_reader_.get_scene_sample_data(scene_info.token);

  // nuScenes sample_data timestamps are not shared across sensors (unlike
  // KITTI's index-aligned files), so an explicit chronological sort is
  // required before writing to the bag.
  std::sort(
    sample_datas.begin(), sample_datas.end(),
    [](const SampleDataInfo & lhs, const SampleDataInfo & rhs) {
      return lhs.timestamp < rhs.timestamp;
    });

  RCLCPP_INFO(
    logger_, "Converting scene '%s' (%zu sample_data entries)...",
    scene_info.name.c_str(), sample_datas.size());

  size_t processed = 0;
  for (const auto & sample_data : sample_datas) {
    const auto calib =
      metadata_reader_.get_calibrated_sensor_info(sample_data.calibrated_sensor_token);
    const auto sensor_name = metadata_reader_.get_sensor_name(calib.sensor_token);
    const std::string channel = to_lower(sensor_name.channel);
    const std::string frame_id = channel + "_link";
    const fs::path file_path = dataroot / sample_data.filename;

    // nuScenes timestamps are microseconds since epoch; rclcpp::Time wants nanoseconds.
    const rclcpp::Time timestamp(sample_data.timestamp * 1000, RCL_ROS_TIME);

    if (sensor_name.modality == "camera") {
      const auto image_msg =
        message_converter_.convert_image_to_msg(file_path, timestamp, frame_id);
      bag_writer_.write_message(image_msg, "/nuscenes/" + channel + "/image_raw", timestamp);

      const auto camera_info_msg = message_converter_.convert_camera_info_to_msg(
        calib, sample_data.width, sample_data.height, timestamp, frame_id);
      bag_writer_.write_message(
        camera_info_msg, "/nuscenes/" + channel + "/camera_info", timestamp);
    } else if (sensor_name.modality == "lidar") {
      const auto lidar_msg =
        message_converter_.convert_lidar_to_msg(file_path, timestamp, frame_id);
      bag_writer_.write_message(lidar_msg, "/nuscenes/" + channel, timestamp);
    } else if (sensor_name.modality == "radar") {
      const auto radar_msg =
        message_converter_.convert_radar_to_msg(file_path, timestamp, frame_id);
      bag_writer_.write_message(radar_msg, "/nuscenes/" + channel, timestamp);
    } else {
      RCLCPP_WARN(
        logger_, "Unknown sensor modality '%s' for channel '%s'; skipping.",
        sensor_name.modality.c_str(), sensor_name.channel.c_str());
    }

    // ego_pose ground truth is written alongside every sample_data, since
    // each sample_data has its own ego_pose computed at that exact timestamp.
    const auto ego_pose = metadata_reader_.get_ego_pose_info(sample_data.ego_pose_token);
    const auto odom_msg = message_converter_.convert_ego_pose_to_odom_msg(
      ego_pose, timestamp, "map", "base_link");
    bag_writer_.write_message(odom_msg, "/nuscenes/odom", timestamp);

    ++processed;
    if (processed % 100 == 0 || processed == sample_datas.size()) {
      RCLCPP_INFO(logger_, "  %zu / %zu", processed, sample_datas.size());
    }
  }

  RCLCPP_INFO(logger_, "Done converting scene '%s'.", scene_info.name.c_str());
}
