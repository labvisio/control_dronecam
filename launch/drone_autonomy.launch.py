import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("control_dronecam"),
        "params.yaml",
    )
    control_cam = Node(
        package="control_dronecam",
        namespace="",
        executable="imgpose_pub",
        name="image_processor",
        parameters=[config],
    )

    init_parrot = Node(
        package="control_dronecam",
        namespace="",
        executable="init_parrot",
        name="drone_init",
    )

    ld = LaunchDescription()
    ld.add_action(control_cam)
    ld.add_action(init_parrot)

    return ld
