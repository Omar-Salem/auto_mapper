import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "map_path",
            description="Map file path",
        ),
        DeclareLaunchArgument(
            "is_sim",
            default_value="true",
        )]

    robot_nodes = create_robot_node()

    return LaunchDescription(declared_arguments +
                             robot_nodes
                             )


def create_robot_node() -> list:
    """

    :rtype: list
    """
    map_path = LaunchConfiguration("map_path")
    is_sim = LaunchConfiguration('is_sim')
    package_name = 'auto_mapper'

    auto_mapper = Node(
        package=package_name,
        executable="auto_mapper",
        name="auto_mapper",
    )

    slam_toolbox_launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'launch', 'online_async_launch.py'])
    slam_params_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "mapper_params_online_async.yaml"])
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file_path),
        launch_arguments={'use_sim_time': is_sim, 'slam_params_file': slam_params_file}.items()
    )

    # map_yaml_file = PathJoinSubstitution([FindPackageShare(package_name), "maps", "apt.yaml"])
    navigation_launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'launch', 'bringup_launch.py'])
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file_path),
        launch_arguments={'map': map_path, 'use_sim_time': is_sim, 'package_name': package_name}.items()
    )
    return [
        # robot_localization,
        GroupAction(
            actions=[
                SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),
                slam_toolbox,
                nav2_bringup,
                auto_mapper,
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="log",
                )
            ]
        )
    ]
