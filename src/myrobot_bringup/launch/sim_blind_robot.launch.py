import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz',
        default_value='true',
        description='Whether to run RViz'
    )

    run_rviz = LaunchConfiguration('run_rviz')


    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("myrobot_bringup"),
            "rviz",
            "nav2_default_view.rviz"
        )],
        condition=IfCondition(run_rviz)
    )

    # Launch Blind Navigation (includes fake_localization)
    navigation_blind = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_navigation"),
            "launch",
            "navigation_blind.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
            "map_name": "eurobot_2026"
        }.items()
    )
        
    return LaunchDescription([
        run_rviz_arg,
        gazebo,
        controller,
        rviz,
        navigation_blind
    ])
