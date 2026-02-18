import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Arguments
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="eurobot_2026",
        description="Name of the map to use (without .yaml extension)"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true"
    )

    # Package directories
    myrobot_navigation_pkg = get_package_share_directory("myrobot_navigation")
    myrobot_localization_pkg = get_package_share_directory("myrobot_localization")

    # Include Fake Localization (starts Map Server + Static TF)
    fake_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(myrobot_localization_pkg, "launch", "fake_localization.launch.py")
        ),
        launch_arguments={
            "map_name": map_name, 
            "use_sim_time": use_sim_time
        }.items()
    )

    # Nav2 Lifecycle Nodes
    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]

    # --- Nav2 Nodes ---

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(myrobot_navigation_pkg, "config", "controller_server_blind.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(myrobot_navigation_pkg, "config", "planner_server_blind.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            os.path.join(myrobot_navigation_pkg, "config", "behavior_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(myrobot_navigation_pkg, "config", "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(myrobot_navigation_pkg, "config", "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
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
        fake_localization_launch,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
    ])
