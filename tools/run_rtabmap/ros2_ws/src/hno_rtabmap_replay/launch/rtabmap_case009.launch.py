import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    database_path = os.environ.get("RTABMAP_DB", "/tmp/hno_case009_rtabmap.db")

    stereo_sync = Node(
        package="rtabmap_sync",
        executable="stereo_sync",
        namespace="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "approx_sync": True,
            "approx_sync_max_interval": 0.02,
        }],
        remappings=[
            ("left/image_rect", "/cam0/image_rect"),
            ("right/image_rect", "/cam1/image_rect"),
            ("left/camera_info", "/cam0/camera_info"),
            ("right/camera_info", "/cam1/camera_info"),
            ("rgbd_image", "rgbd_image"),
        ],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "subscribe_rgbd": True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "publish_tf": True,
            "publish_tf_map": True,
            "database_path": database_path,
            "wait_for_transform": 0.2,
            "Reg/Force3DoF": "false",
            "RGBD/OptimizeFromGraphEnd": "true",
            "RGBD/NeighborLinkRefining": "true",
            "Vis/MinInliers": "12",
            "Kp/MaxFeatures": "800",
        }],
        remappings=[
            ("rgbd_image", "rgbd_image"),
        ],
        arguments=["--delete_db_on_start"],
    )

    return LaunchDescription([stereo_sync, rtabmap])
