import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )


    navigation_blind = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_navigation"),
            "launch",
            "navigation_blind.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_name": "eurobot_2026"
        }.items()
    )
        
    return LaunchDescription([
        use_sim_time_arg,
        hardware_interface,
        controller,
        navigation_blind,
    ])
