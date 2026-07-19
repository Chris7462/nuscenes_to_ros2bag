# nuScenes to ROS 2 Bag

Convert a single [nuScenes](https://www.nuscenes.org/) scene into a ROS 2 bag (MCAP format).

Unlike the original [nuscenes2bag](https://github.com/clynamen/nuscenes2bag) tool (ROS 1, batch/multi-scene,
multi-threaded), this package converts exactly **one scene per run**, following the design of
[kitti_to_ros2bag](https://github.com/Chris7462/kitti_to_ros2bag). To convert multiple scenes, invoke this
node once per scene (e.g. from a shell script), optionally in parallel.

Sensor extrinsics are **not** written to the bag. They live in the sibling
[nuscenes_urdf](../nuscenes_urdf) package instead, published via `robot_state_publisher`. This node only
writes raw sensor data and the `ego_pose` ground-truth trajectory (as `nav_msgs/msg/Odometry` on
`/nuscenes/odom`).

## How to use it?

1. Download the [nuScenes dataset](https://www.nuscenes.org/nuscenes#download) and unzip it, so that
   `{dataroot}/{version}/*.json` (the metadata tables) and `{dataroot}/samples`, `{dataroot}/sweeps` exist.
2. Clone this repo, and its dependency [nuscenes_msgs](../nuscenes_msgs), under your
   **{ros2_workspace}/src** folder.
3. Modify the **nuscenes_to_ros2bag.yaml** file under the *param* folder.
    * *dataroot*: path to the root of the dataset (containing `maps`, `samples`, `sweeps`, and the
      `{version}` metadata folder)
    * *version*: metadata sub-directory, e.g. `"v1.0-mini"`
    * *scene_number*: the numeric scene id to convert, e.g. `61` for `scene-0061`

    For example:

    ```yaml
    nuscenes_to_ros2bag_node:
      ros__parameters:
        dataroot: "/data/nuscenes"
        version: "v1.0-mini"
        scene_number: 61
    ```
4. Build the package
    ```bash
    colcon build --symlink-install --packages-select nuscenes_to_ros2bag
    ```
5. Launch the package
    ```bash
    source ./install/setup.bash
    ros2 launch nuscenes_to_ros2bag nuscenes_to_ros2bag_launch.py
    ```
    That's it. You have the file *scene-0061_bag* that contains your data.

## Topics written

| Topic | Type |
|---|---|
| `/nuscenes/cam_front/image_raw` (x6, one per camera) | `sensor_msgs/msg/Image` |
| `/nuscenes/cam_front/camera_info` (x6, one per camera) | `sensor_msgs/msg/CameraInfo` |
| `/nuscenes/lidar_top` | `sensor_msgs/msg/PointCloud2` |
| `/nuscenes/radar_front` (x5, one per radar) | `nuscenes_msgs/msg/RadarObjects` |
| `/nuscenes/odom` | `nav_msgs/msg/Odometry` |

Frame IDs on every message (e.g. `cam_front_link`, `lidar_top_link`, `radar_front_link`) match the link
names in [nuscenes_urdf](../nuscenes_urdf), so the bag lines up with the published TF tree with no
remapping needed.

## Design notes

* **Single scene per run, no threading.** Batch conversion of an entire dataset is left to an external
  wrapper script that invokes this node once per scene, rather than being built into the node itself.
* **Eager, single-pass conversion.** All of a scene's `sample_data` entries are sorted by timestamp once
  and written in order inside the node's constructor -- there is no timer-driven pacing, since this is an
  offline batch tool with no real-time playback requirement (this can be revisited later if a live-replay
  use case comes up).
* **No TF in the bag.** Static sensor-to-`base_link` transforms are published separately by
  [nuscenes_urdf](../nuscenes_urdf) via `robot_state_publisher`, mirroring how `kitti_urdf` works
  alongside `kitti_to_ros2bag`.
* **Lidar** points are read as `(x, y, z, intensity, ring)` float32 quintuples, matching nuScenes'
  `LIDAR_TOP` `.pcd.bin` format.
* **Radar** points use the custom `nuscenes_msgs/msg/RadarObject(s)` message, preserving every raw field
  nuScenes provides (including quality/validity flags with no standard ROS message equivalent), parsed
  directly from the radar `.pcd` files' 43-byte binary point records.

## Related packages

* [nuscenes_msgs](../nuscenes_msgs) -- custom `RadarObject`/`RadarObjects` message definitions
* [nuscenes_urdf](../nuscenes_urdf) -- sensor rig URDF + `robot_state_publisher` launch
