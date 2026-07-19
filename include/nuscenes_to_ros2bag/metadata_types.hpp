#pragma once

// C++
#include <array>
#include <cstdint>
#include <string>
#include <vector>

using Token = std::string;
using TimeStamp = int64_t;  // microseconds since epoch (nuScenes convention)

struct SceneInfo
{
  Token token;
  uint32_t sample_number;
  uint32_t scene_id;
  std::string name;
  std::string description;
  Token first_sample_token;
};

struct SampleDataInfo
{
  Token token;
  Token sample_token;
  TimeStamp timestamp;
  Token ego_pose_token;
  Token calibrated_sensor_token;
  std::string fileformat;
  bool is_key_frame;
  std::string filename;
  uint32_t width;   // meaningful for camera modality only; 0 otherwise
  uint32_t height;  // meaningful for camera modality only; 0 otherwise
};

struct EgoPoseInfo
{
  Token token;
  TimeStamp timestamp;
  std::array<double, 3> translation;
  std::array<double, 4> rotation;  // (w, x, y, z), nuScenes convention
};

struct CalibratedSensorInfo
{
  Token token;
  Token sensor_token;
  std::array<double, 3> translation;
  std::array<double, 4> rotation;  // (w, x, y, z)
  std::vector<std::vector<double>> camera_intrinsic;  // empty for non-camera sensors
};

struct CalibratedSensorName
{
  Token token;  // sensor_token
  std::string channel;   // e.g. "CAM_FRONT"
  std::string modality;  // "camera" | "lidar" | "radar"
};
