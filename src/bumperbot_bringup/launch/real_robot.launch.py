import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("bumperbot_description"),
            "rviz",
            "display.rviz"
        )],
    )

    twist_converter = Node(
        package="bumperbot_firmware",
        executable="velocityconverter.py",
        name="velocity_converter",
        output="screen",
    )
        
    return LaunchDescription([
        hardware_interface,
        controller,
        twist_converter,
        rviz
    ])
