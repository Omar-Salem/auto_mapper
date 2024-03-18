```bash
clear && colcon build --packages-select auto_mapper && source install/local_setup.bash && ros2 launch auto_mapper auto_mapper.launch.py map_path:=~/map
```