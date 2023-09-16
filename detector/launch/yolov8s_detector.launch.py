from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera = Node(
        package="detector",
        executable="camera"
    )

    detector = Node(
        package="detector",
        executable="detector"
    )

    vizualizator = Node(
        package="objects_vis",
        executable="draw_objects"
    )

    ld.add_action(camera)
    ld.add_action(detector)
    ld.add_action(vizualizator)

    return ld