import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or default will be used
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="anafi", description="Namespace for this Anafi"
    )
    ip_arg = DeclareLaunchArgument(
        "ip",
        default_value="192.168.53.1",  # Anafi: '192.168.42.1', SkyController: '192.168.53.1', Sphinx: '10.202.0.1'
        description="IP address of the device",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="ai",  # {'4k', 'thermal', 'usa', 'ai'}
        description="Model of the drone",
    )

    anafi_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("anafi_ros_nodes")),
                "/anafi_launch.py",
            ]
        ),
        launch_arguments={
            "drone/model": LaunchConfiguration("model"),
            "device/ip": LaunchConfiguration("ip"),
        }.items(),
    )

    pub_tf = Node(
        package="control_dronecam",
        namespace="",
        executable="pub_tf",
        name="tf_publisher",
    )

    return LaunchDescription([namespace_arg, ip_arg, model_arg, anafi_include, pub_tf])
