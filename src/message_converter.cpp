// C++ Standard Library
#include <cstdint>
#include <fstream>
#include <sstream>

// OpenCV
#include <opencv2/imgcodecs.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS 2
#include <cv_bridge/cv_bridge.hpp>

// local
#include "nuscenes_to_ros2bag/message_converter.hpp"


// nuScenes LIDAR_TOP points are (x, y, z, intensity, ring), all float32.
// This mirrors the standard "Velodyne-style" XYZIR point used by most ROS
// lidar drivers, but is defined locally here since it isn't part of any
// PCL/ROS message package.
struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  float ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIR,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, ring, ring))


MessageConverter::MessageConverter(const rclcpp::Logger & logger)
: logger_(logger)
{
}

sensor_msgs::msg::Image MessageConverter::convert_image_to_msg(
  const fs::path & file_path, const rclcpp::Time & timestamp, const std::string & frame_id)
{
  cv::Mat image = cv::imread(file_path, cv::IMREAD_COLOR);

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
  msg->header.frame_id = frame_id;
  msg->header.stamp = timestamp;

  return *msg;
}

sensor_msgs::msg::CameraInfo MessageConverter::convert_camera_info_to_msg(
  const CalibratedSensorInfo & calib, uint32_t width, uint32_t height,
  const rclcpp::Time & timestamp, const std::string & frame_id)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = timestamp;
  msg.width = width;
  msg.height = height;

  // nuScenes camera images are already undistorted and rectified, so the
  // dataset provides no distortion model or coefficients.
  msg.distortion_model = "";

  if (calib.camera_intrinsic.size() == 3) {
    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        msg.k[row * 3 + col] = calib.camera_intrinsic[row][col];
      }
    }
  } else {
    RCLCPP_WARN(
      logger_, "camera_intrinsic for frame_id '%s' is not a 3x3 matrix; CameraInfo.k left at zero.",
      frame_id.c_str());
  }

  // Identity rectification and a projection matrix with no stereo
  // baseline, consistent with a single already-rectified monocular camera.
  msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.p = {
    msg.k[0], msg.k[1], msg.k[2], 0.0,
    msg.k[3], msg.k[4], msg.k[5], 0.0,
    msg.k[6], msg.k[7], msg.k[8], 0.0};

  return msg;
}

sensor_msgs::msg::PointCloud2 MessageConverter::convert_lidar_to_msg(
  const fs::path & file_path, const rclcpp::Time & timestamp, const std::string & frame_id)
{
  pcl::PointCloud<PointXYZIR> cloud;

  std::ifstream input_file(file_path, std::ios::binary);
  if (!input_file.is_open()) {
    RCLCPP_ERROR(logger_, "Unable to open lidar file %s", file_path.c_str());
  } else {
    while (input_file.good() && !input_file.eof()) {
      PointXYZIR point;
      input_file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
      input_file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
      input_file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
      input_file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
      input_file.read(reinterpret_cast<char *>(&point.ring), sizeof(float));
      if (input_file) {
        cloud.push_back(point);
      }
    }
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = timestamp;

  return msg;
}

nuscenes_msgs::msg::RadarObjects MessageConverter::convert_radar_to_msg(
  const fs::path & file_path, const rclcpp::Time & timestamp, const std::string & frame_id)
{
  nuscenes_msgs::msg::RadarObjects msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = timestamp;

  std::ifstream input_file(file_path, std::ios::binary);
  if (!input_file.is_open()) {
    RCLCPP_ERROR(logger_, "Unable to open radar file %s", file_path.c_str());
    return msg;
  }

  // Parse the short ASCII PCD header line by line. We only need the point
  // count (WIDTH); binary point data starts immediately after the line
  // beginning with "DATA".
  std::string line;
  size_t num_points = 0;
  while (std::getline(input_file, line)) {
    if (line.rfind("WIDTH", 0) == 0) {
      std::istringstream iss(line);
      std::string key;
      iss >> key >> num_points;
    }
    if (line.rfind("DATA", 0) == 0) {
      break;
    }
  }

  // Each radar point is a fixed 43-byte binary record, per the nuScenes
  // radar .pcd header:
  //   FIELDS: x y z dyn_prop id rcs vx vy vx_comp vy_comp is_quality_valid
  //           ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms
  //   SIZE:   4 4 4 1        2  4   4  4  4       4       1
  //           1           1     1     1              1     1       1
  msg.objects.reserve(num_points);
  for (size_t i = 0; i < num_points && input_file.good(); ++i) {
    nuscenes_msgs::msg::RadarObject obj;

    float x = 0.0f, y = 0.0f, z = 0.0f;
    input_file.read(reinterpret_cast<char *>(&x), sizeof(float));
    input_file.read(reinterpret_cast<char *>(&y), sizeof(float));
    input_file.read(reinterpret_cast<char *>(&z), sizeof(float));
    obj.pose.x = x;
    obj.pose.y = y;
    obj.pose.z = z;

    input_file.read(reinterpret_cast<char *>(&obj.dyn_prop), sizeof(uint8_t));
    input_file.read(reinterpret_cast<char *>(&obj.id), sizeof(uint16_t));
    input_file.read(reinterpret_cast<char *>(&obj.rcs), sizeof(float));
    input_file.read(reinterpret_cast<char *>(&obj.vx), sizeof(float));
    input_file.read(reinterpret_cast<char *>(&obj.vy), sizeof(float));
    input_file.read(reinterpret_cast<char *>(&obj.vx_comp), sizeof(float));
    input_file.read(reinterpret_cast<char *>(&obj.vy_comp), sizeof(float));
    input_file.read(reinterpret_cast<char *>(&obj.is_quality_valid), sizeof(uint8_t));
    input_file.read(reinterpret_cast<char *>(&obj.ambig_state), sizeof(uint8_t));
    input_file.read(reinterpret_cast<char *>(&obj.x_rms), sizeof(uint8_t));
    input_file.read(reinterpret_cast<char *>(&obj.y_rms), sizeof(uint8_t));
    input_file.read(reinterpret_cast<char *>(&obj.invalid_state), sizeof(uint8_t));
    input_file.read(reinterpret_cast<char *>(&obj.pdh0), sizeof(uint8_t));
    input_file.read(reinterpret_cast<char *>(&obj.vx_rms), sizeof(uint8_t));
    input_file.read(reinterpret_cast<char *>(&obj.vy_rms), sizeof(uint8_t));

    msg.objects.push_back(obj);
  }

  return msg;
}

nav_msgs::msg::Odometry MessageConverter::convert_ego_pose_to_odom_msg(
  const EgoPoseInfo & ego_pose, const rclcpp::Time & timestamp,
  const std::string & frame_id, const std::string & child_frame_id)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = timestamp;
  msg.child_frame_id = child_frame_id;

  msg.pose.pose.position.x = ego_pose.translation[0];
  msg.pose.pose.position.y = ego_pose.translation[1];
  msg.pose.pose.position.z = ego_pose.translation[2];

  // nuScenes stores rotation as (w, x, y, z); geometry_msgs expects (x, y, z, w).
  msg.pose.pose.orientation.w = ego_pose.rotation[0];
  msg.pose.pose.orientation.x = ego_pose.rotation[1];
  msg.pose.pose.orientation.y = ego_pose.rotation[2];
  msg.pose.pose.orientation.z = ego_pose.rotation[3];

  return msg;
}
