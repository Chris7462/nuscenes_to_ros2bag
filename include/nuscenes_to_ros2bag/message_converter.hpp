#pragma once

// C++ Standard Library
#include <filesystem>
#include <string>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// nuscenes_msgs
#include <nuscenes_msgs/msg/radar_objects.hpp>

// local
#include "nuscenes_to_ros2bag/metadata_types.hpp"

namespace fs = std::filesystem;

class MessageConverter
{
public:
  explicit MessageConverter(const rclcpp::Logger & logger);

  sensor_msgs::msg::Image convert_image_to_msg(
    const fs::path & file_path, const rclcpp::Time & timestamp, const std::string & frame_id);

  // width/height come from the sample_data entry (nuScenes only records
  // these for camera modality); camera_intrinsic comes from calibrated_sensor.
  sensor_msgs::msg::CameraInfo convert_camera_info_to_msg(
    const CalibratedSensorInfo & calib, uint32_t width, uint32_t height,
    const rclcpp::Time & timestamp, const std::string & frame_id);

  // nuScenes LIDAR_TOP .pcd.bin files have no header: a flat array of
  // float32 quintuples (x, y, z, intensity, ring).
  sensor_msgs::msg::PointCloud2 convert_lidar_to_msg(
    const fs::path & file_path, const rclcpp::Time & timestamp, const std::string & frame_id);

  // nuScenes radar .pcd files have a short ASCII PCD header followed by
  // fixed-size 43-byte binary records (18 fields per point).
  nuscenes_msgs::msg::RadarObjects convert_radar_to_msg(
    const fs::path & file_path, const rclcpp::Time & timestamp, const std::string & frame_id);

  // ego_pose is nuScenes' fused ground-truth vehicle pose in the log's
  // global (map) frame. No TF is broadcast here -- see nuscenes_urdf for
  // the static sensor-to-base_link transforms.
  nav_msgs::msg::Odometry convert_ego_pose_to_odom_msg(
    const EgoPoseInfo & ego_pose, const rclcpp::Time & timestamp,
    const std::string & frame_id, const std::string & child_frame_id);

private:
  rclcpp::Logger logger_;
};
