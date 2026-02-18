from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["myrobot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )



    twist_relay= Node(
        package="myrobot_controller",
        executable="twist_relay.py",
        name="twist_relay",
    )
    
    return LaunchDescription(
        [   
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            twist_relay
        ]
    )