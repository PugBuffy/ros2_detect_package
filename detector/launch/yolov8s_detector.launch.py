from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera = Node(
        package="detector",
        executable="camera", 
        # parameters=[
        #      {'video_file':'/dev/video0'}
        #  ]
    )

    detector = Node(
        package="detector",
        executable="detector"
    )

    # tracker = Node(
    #     package="bytetrack_cpp_node",
    #     executable='bytetrack_cpp_node'
    # )

    vizualizator = Node(
        package="objects_vis",
        executable="draw_objects"
    )



    ld.add_action(camera)
    ld.add_action(detector)
    # ld.add_action(tracker)
    ld.add_action(vizualizator)

    return ld