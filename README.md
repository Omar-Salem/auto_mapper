<h3 align="center">auto_mapper</h3>

  <p align="center">
    Package for auto exploring frontiers and generating a map with ROS 2 Humble / Iron
  </p>

<video src="https://github.com/Omar-Salem/auto_mapper/blob/master/Demo.mp4" width="320" height="240" controls></video>

### Prerequisites

* [ROS 2 Humble / Iron](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your Ubuntu 22.04
* [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
* [nav2](https://github.com/ros-planning/navigation2)
* [rviz2](https://github.com/ros2/rviz)

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
ros2 launch auto_mapper auto_mapper.launch.py map_path:=~/map
```

5. Launch turtlebot3
```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```


### Gotchas
* If you receive `Failed to write file: Exception output stream error`, change the map file path to a dir where write access is granted.
* To use with a custom robot, or if the Twist messages aren't published to `/cmd_vel`, uncomment `SetRemap(src='/cmd_vel', dst='{/your/custom_cmd_vel}}')` in `auto_mapper.launch.py`

### Contact

Omar Salem - [LinkedIn](https://www.linkedin.com/in/omar-salem-4564a590/)



### Acknowledgements
* [m-explore](https://github.com/hrnr/m-explore)
