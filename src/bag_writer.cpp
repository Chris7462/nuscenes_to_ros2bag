// C++
#include <array>
#include <string>

// ROS 2
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_cpp/converter_options.hpp>

// local
#include "nuscenes_to_ros2bag/bag_writer.hpp"


BagWriter::BagWriter(const std::string & output_bag_name)
{
  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  // Configure storage options for MCAP format
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_bag_name;
  storage_options.storage_id = "mcap";

  // Configure converter options
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  writer_->open(storage_options, converter_options);
}

void BagWriter::create_topics()
{
  // SensorDataQoS: best_effort, volatile, keep_last(5)
  // Used for high-bandwidth streams where dropping old frames is acceptable.
  const std::vector<rclcpp::QoS> sensor_qos = {rclcpp::SensorDataQoS().keep_last(5)};

  // Reliable QoS, keep_last(10)
  // Used for low-rate metadata (camera_info, odom) where every message matters.
  const std::vector<rclcpp::QoS> reliable_qos = {
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile()};

  // Camera topics (image + camera_info per camera)
  const std::array<std::string, 6> cameras = {
    "cam_front", "cam_front_left", "cam_front_right",
    "cam_back", "cam_back_left", "cam_back_right"};
  for (const auto & cam : cameras) {
    create_topic("/nuscenes/" + cam + "/image_raw", "sensor_msgs/msg/Image", sensor_qos);
    create_topic("/nuscenes/" + cam + "/camera_info", "sensor_msgs/msg/CameraInfo", reliable_qos);
  }

  // Lidar topic
  create_topic("/nuscenes/lidar_top", "sensor_msgs/msg/PointCloud2", sensor_qos);

  // Radar topics
  const std::array<std::string, 5> radars = {
    "radar_front", "radar_front_left", "radar_front_right",
    "radar_back_left", "radar_back_right"};
  for (const auto & radar : radars) {
    create_topic("/nuscenes/" + radar, "nuscenes_msgs/msg/RadarObjects", sensor_qos);
  }

  // Ego pose (ground truth)
  create_topic("/nuscenes/odom", "nav_msgs/msg/Odometry", reliable_qos);
}

void BagWriter::create_topic(
  const std::string & topic_name,
  const std::string & topic_type,
  const std::vector<rclcpp::QoS> & qos_profiles)
{
  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic_name;
  topic_metadata.type = topic_type;
  topic_metadata.serialization_format = "cdr";
  topic_metadata.offered_qos_profiles = qos_profiles;
  // type_description_hash is a required field added in post-Jazzy rosbag2.
  // Empty string is valid when not recording live type introspection data.
  topic_metadata.type_description_hash = "";
  writer_->create_topic(topic_metadata);
}
