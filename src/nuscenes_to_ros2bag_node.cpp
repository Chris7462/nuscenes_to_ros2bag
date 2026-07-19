#include "nuscenes_to_ros2bag/nuscenes_to_ros2bag.hpp"


int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NuscenesToRos2bagNode>();
  rclcpp::shutdown();

  return 0;
}
