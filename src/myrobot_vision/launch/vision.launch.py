import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot_vision')
    config_file = os.path.join(pkg_share, 'config', 'vision_settings.yaml')
    markers_file = os.path.join(pkg_share, 'config', 'markers.yaml')

    return LaunchDescription([
        Node(
            package='myrobot_vision',
            executable='aruco_detector.py',
            name='aruco_detector',
            parameters=[{'config_yaml': config_file}],
            output='screen'
        ),
        Node(
            package='myrobot_vision',
            executable='detected_crates_tf.py',
            name='detected_crates_tf',
            output='screen'
        ),
        Node(
            package='myrobot_vision',
            executable='relocalizer.py',
            name='aruco_relocalizer',
            parameters=[{'markers_config': markers_file}],
            output='screen'
        ),
        Node(
            package='myrobot_vision',
            executable='sequence_manager.py',
            name='sequence_manager',
            output='screen'
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            arguments=['/front_camera/image_raw'],
            output='screen'
        )
    ])
