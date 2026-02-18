import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ["map_server"]

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="eurobot_2026"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    map_path = PathJoinSubstitution([
        get_package_share_directory("myrobot_mapping"),
        "maps",
        PathJoinSubstitution([map_name, ".yaml"]) # This might need fix depending on how map_name is used
    ])
    
    # Correctly handle map file path with extension
    # We use PythonExpression to ensure .yaml is appended even if map_name comes from launch config
    map_yaml_file = PathJoinSubstitution([
        get_package_share_directory("myrobot_mapping"),
        "maps",
        PythonExpression(["'", map_name, ".yaml'"])
    ])
    
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml_file}, # Hardcoded for reliability in this specific case
            {"use_sim_time": use_sim_time}
        ],
    )

    # Static transform between map and odom (Fake Localization)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        nav2_map_server,
        static_tf_node,
        nav2_lifecycle_manager,
    ])
