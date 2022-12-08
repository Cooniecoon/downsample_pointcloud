from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="downsample_pointcloud",
            executable="downsampler",
            name="front_bottom",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"subscribe_topic_name": "/front_bottom_astra/depth/points"},
                {"publish_topic_name": "/front_bottom_astra/depth/points/downsampled"}
            ]
        ),
        Node(
            package="downsample_pointcloud",
            executable="downsampler",
            name="front_top",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"subscribe_topic_name": "/front_top_astra/depth/points"},
                {"publish_topic_name": "/front_top_astra/depth/points/downsampled"}
            ]
        ),
        Node(
            package="downsample_pointcloud",
            executable="downsampler",
            name="rear",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"subscribe_topic_name": "/rear/depth/points"},
                {"publish_topic_name": "/rear/depth/points/downsampled"}
            ]
        ),
    ])
