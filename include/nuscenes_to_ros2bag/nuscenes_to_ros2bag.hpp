#pragma once

// ROS 2
#include <rclcpp/rclcpp.hpp>

// This node performs a single eager, single-pass conversion of one nuScenes
// scene entirely inside its constructor -- unlike kitti_to_ros2bag's
// timer-driven Kitti2BagNode, there is no per-frame pacing and no member
// state needs to persist across callbacks, so no helper objects or timer
// are kept as members here.
class NuscenesToRos2bagNode : public rclcpp::Node
{
public:
  NuscenesToRos2bagNode();
};
