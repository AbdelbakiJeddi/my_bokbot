import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz',
        default_value='true',
        description='Whether to run RViz'
    )

    run_rviz = LaunchConfiguration('run_rviz')
    use_slam = LaunchConfiguration("use_slam")


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

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )
        
    return LaunchDescription([
        run_rviz_arg,
        use_slam_arg,
        gazebo,
        controller,
        rviz
    ])
