import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Declare Arguments
    is_sim_arg = DeclareLaunchArgument(
        'is_sim', default_value='true', description='Whether to run in simulation (Gazebo)'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true', description='Whether to run Gazebo without GUI'
    )
    use_vision_arg = DeclareLaunchArgument(
        'use_vision', default_value='true', description='Whether to run ArUco vision nodes'
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='arena_world', description='Gazebo world file name'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='eurobot_2026', description='Map name for navigation'
    )

    # 2. Substitutions
    is_sim = LaunchConfiguration('is_sim')
    headless = LaunchConfiguration('headless')
    use_vision = LaunchConfiguration('use_vision')
    world_name = LaunchConfiguration('world_name')
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('is_sim') # Simulation clock if is_sim is true

    # 3. Simulation Components (Only if is_sim is true)
    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("myrobot_description"), "launch", "gazebo.launch.py"),
        launch_arguments={
            'world_name': world_name,
            'headless': headless
        }.items(),
        condition=IfCondition(is_sim)
    )

    # 4. Hardware Components (Only if is_sim is false)
    hardware_interface = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("myrobot_firmware"), "launch", "hardware_interface.launch.py"),
        condition=UnlessCondition(is_sim)
    )

    # 5. Core Components (Always run)
    controller = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("myrobot_controller"), "launch", "controller.launch.py"),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    navigation = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("myrobot_navigation"), "launch", "navigation_blind.launch.py"),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_name': map_name
        }.items()
    )

    # 6. Vision Components (Optional)
    vision = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("myrobot_vision"), "launch", "vision.launch.py"),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_vision)
    )

    # 7. Visualization (Optional, maybe add an argument later)
    # rviz = Node(...)

    return LaunchDescription([
        is_sim_arg,
        headless_arg,
        use_vision_arg,
        world_name_arg,
        map_name_arg,
        gazebo,
        hardware_interface,
        controller,
        navigation,
        vision
    ])
