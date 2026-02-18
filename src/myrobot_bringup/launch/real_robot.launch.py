import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("myrobot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen",
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    #imu_driver_node = Node(
    #    package="myrobot_firmware",
    #    executable="mpu6050_driver.py"
    #)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("myrobot_description"),
            "rviz",
            "display.rviz"
        )],
        condition=IfCondition(run_rviz)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("myrobot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )
        
    return LaunchDescription([

        run_rviz_arg,
        use_slam_arg,
        hardware_interface,
        #laser_driver,
        controller,
        #imu_driver_node,
        #slam,
        #rviz
    ])
