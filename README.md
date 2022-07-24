# aw_universe
autoware universe build


```bash
cd autoware
mkdir src
vcs import src < autoware.repos

# planning_simulator
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/aw_universe/maps/town10 vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

# pointcloud_map_loader
ros2 run map_loader pointcloud_map_loader --ros-args -p "pcd_paths_or_directory:=[path/to/pointcloud1.pcd, path/to/pointcloud2.pcd, ...]"`

# lanelet2_map_loader
ros2 run map_loader lanelet2_map_loader --ros-args -p lanelet2_map_path:=path/to/map.osm

# lanelet2_map_visualization
ros2 run map_loader lanelet2_map_visualization

# autoware-universe:latest-prebuilt
docker pull ghcr.io/autowarefoundation/autoware-universe:latest-prebuilt
```

## メモ
- [colcon build の並列数を減らすとbuildできた。](https://zenn.dev/iwatake2222/scraps/1a8009020b8fe1)
- フォルダ配置
  - maps
    - lanelet2_map.osm
    - pointcloud_map.pcd
    - map_config.yaml
