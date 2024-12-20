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
                {"exposuretime": 3000.00},
                {"gain":8.00}
            ]
        )
    ])