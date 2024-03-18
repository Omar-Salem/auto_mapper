<h3 align="center">auto_mapper</h3>

  <p align="center">
    Package for auto exploring frontiers with ROS 2 Humble / Iron
  </p>

### Prerequisites

* [ROS 2 Humble / Iron](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your Ubuntu 22.04
* [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
* [nav2](https://github.com/ros-planning/navigation2)

### Installation & usage

1. Clone the repo
```sh
git clone https://github.com/Omar-Salem/auto_mapper
```
2. Build the ROS 2  workspace
```sh
cd {workspace_dir}
```
```sh
colcon build --packages-select auto_mapper
```
3. Source the ROS Workspace
```sh
. install/setup.bash
```

4. Launch the mapper
```sh
. ros2 launch auto_mapper auto_mapper.launch.py map_path:=~/map
```
