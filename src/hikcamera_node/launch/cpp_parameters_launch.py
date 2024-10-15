from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hikcamera_node",
            executable="camera_node",
            name="custom_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"exposuretime": 4000.00},
                {"gain":16.00}
            ]
        )
    ])