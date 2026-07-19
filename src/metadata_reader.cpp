// C++
#include <algorithm>
#include <fstream>
#include <stdexcept>

// local
#include "nuscenes_to_ros2bag/metadata_reader.hpp"


MetadataReader::MetadataReader(const rclcpp::Logger & logger)
: logger_(logger)
{
}

nlohmann::json MetadataReader::slurp_json_file(const fs::path & file_path)
{
  std::ifstream file(file_path.string());
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open " + file_path.string());
  }
  nlohmann::json json_data;
  file >> json_data;
  return json_data;
}

void MetadataReader::load_from_directory(const fs::path & metadata_path)
{
  load_scenes(metadata_path / "scene.json");
  load_samples(metadata_path / "sample.json");
  load_sample_data(metadata_path / "sample_data.json");
  load_ego_pose(metadata_path / "ego_pose.json");
  load_calibrated_sensor(metadata_path / "calibrated_sensor.json");
  load_sensor(metadata_path / "sensor.json");
}

void MetadataReader::load_scenes(const fs::path & file_path)
{
  const auto scene_jsons = slurp_json_file(file_path);
  scenes_.reserve(scene_jsons.size());

  for (const auto & scene_json : scene_jsons) {
    // scene "name" looks like "scene-0061"; the numeric id is what the
    // scene_number parameter refers to.
    const std::string name = scene_json.at("name").get<std::string>();
    const auto dash_pos = name.find('-');
    const uint32_t scene_id = static_cast<uint32_t>(std::stoul(name.substr(dash_pos + 1)));

    scenes_.push_back(SceneInfo{
      scene_json.at("token").get<Token>(),
      scene_json.at("nbr_samples").get<uint32_t>(),
      scene_id,
      name,
      scene_json.at("description").get<std::string>(),
      scene_json.at("first_sample_token").get<Token>()});
  }
}

void MetadataReader::load_samples(const fs::path & file_path)
{
  const auto sample_jsons = slurp_json_file(file_path);

  for (const auto & sample_json : sample_jsons) {
    sample_token_to_scene_token_[sample_json.at("token").get<Token>()] =
      sample_json.at("scene_token").get<Token>();
  }
}

void MetadataReader::load_sample_data(const fs::path & file_path)
{
  const auto sample_data_jsons = slurp_json_file(file_path);
  sample_data_.reserve(sample_data_jsons.size());

  for (const auto & sd : sample_data_jsons) {
    SampleDataInfo info;
    info.token = sd.at("token").get<Token>();
    info.sample_token = sd.at("sample_token").get<Token>();
    info.timestamp = sd.at("timestamp").get<TimeStamp>();
    info.ego_pose_token = sd.at("ego_pose_token").get<Token>();
    info.calibrated_sensor_token = sd.at("calibrated_sensor_token").get<Token>();
    info.fileformat = sd.at("fileformat").get<std::string>();
    info.is_key_frame = sd.at("is_key_frame").get<bool>();
    info.filename = sd.at("filename").get<std::string>();
    // width/height are only present for camera sample_data entries.
    info.width = sd.value("width", 0);
    info.height = sd.value("height", 0);
    sample_data_.push_back(info);
  }
}

void MetadataReader::load_ego_pose(const fs::path & file_path)
{
  const auto ego_pose_jsons = slurp_json_file(file_path);

  for (const auto & ep : ego_pose_jsons) {
    EgoPoseInfo info;
    info.token = ep.at("token").get<Token>();
    info.timestamp = ep.at("timestamp").get<TimeStamp>();

    const auto & t = ep.at("translation");
    info.translation = {t[0].get<double>(), t[1].get<double>(), t[2].get<double>()};

    const auto & r = ep.at("rotation");
    info.rotation = {
      r[0].get<double>(), r[1].get<double>(), r[2].get<double>(), r[3].get<double>()};

    ego_pose_by_token_.emplace(info.token, info);
  }
}

void MetadataReader::load_calibrated_sensor(const fs::path & file_path)
{
  const auto calibrated_sensor_jsons = slurp_json_file(file_path);

  for (const auto & cs : calibrated_sensor_jsons) {
    CalibratedSensorInfo info;
    info.token = cs.at("token").get<Token>();
    info.sensor_token = cs.at("sensor_token").get<Token>();

    const auto & t = cs.at("translation");
    info.translation = {t[0].get<double>(), t[1].get<double>(), t[2].get<double>()};

    const auto & r = cs.at("rotation");
    info.rotation = {
      r[0].get<double>(), r[1].get<double>(), r[2].get<double>(), r[3].get<double>()};

    // Empty for lidar/radar; a 3x3 matrix for cameras.
    if (cs.contains("camera_intrinsic")) {
      for (const auto & row : cs.at("camera_intrinsic")) {
        std::vector<double> row_vec;
        row_vec.reserve(row.size());
        for (const auto & value : row) {
          row_vec.push_back(value.get<double>());
        }
        info.camera_intrinsic.push_back(std::move(row_vec));
      }
    }

    calibrated_sensor_by_token_.emplace(info.token, info);
  }
}

void MetadataReader::load_sensor(const fs::path & file_path)
{
  const auto sensor_jsons = slurp_json_file(file_path);

  for (const auto & s : sensor_jsons) {
    CalibratedSensorName info;
    info.token = s.at("token").get<Token>();
    info.channel = s.at("channel").get<std::string>();
    info.modality = s.at("modality").get<std::string>();
    sensor_name_by_token_.emplace(info.token, info);
  }
}

std::optional<SceneInfo> MetadataReader::get_scene_info_by_number(uint32_t scene_number) const
{
  const auto it = std::find_if(
    scenes_.begin(), scenes_.end(),
    [scene_number](const SceneInfo & scene) {return scene.scene_id == scene_number;});

  if (it == scenes_.end()) {
    return std::nullopt;
  }
  return *it;
}

std::vector<SampleDataInfo> MetadataReader::get_scene_sample_data(const Token & scene_token) const
{
  std::vector<SampleDataInfo> result;

  for (const auto & sample_data : sample_data_) {
    const auto it = sample_token_to_scene_token_.find(sample_data.sample_token);
    if (it != sample_token_to_scene_token_.end() && it->second == scene_token) {
      result.push_back(sample_data);
    }
  }

  return result;
}

EgoPoseInfo MetadataReader::get_ego_pose_info(const Token & ego_pose_token) const
{
  const auto it = ego_pose_by_token_.find(ego_pose_token);
  if (it == ego_pose_by_token_.end()) {
    throw std::runtime_error("Unable to find ego_pose for token " + ego_pose_token);
  }
  return it->second;
}

CalibratedSensorInfo MetadataReader::get_calibrated_sensor_info(
  const Token & calibrated_sensor_token) const
{
  const auto it = calibrated_sensor_by_token_.find(calibrated_sensor_token);
  if (it == calibrated_sensor_by_token_.end()) {
    throw std::runtime_error(
      "Unable to find calibrated_sensor for token " + calibrated_sensor_token);
  }
  return it->second;
}

CalibratedSensorName MetadataReader::get_sensor_name(const Token & sensor_token) const
{
  const auto it = sensor_name_by_token_.find(sensor_token);
  if (it == sensor_name_by_token_.end()) {
    throw std::runtime_error("Unable to find sensor for token " + sensor_token);
  }
  return it->second;
}
