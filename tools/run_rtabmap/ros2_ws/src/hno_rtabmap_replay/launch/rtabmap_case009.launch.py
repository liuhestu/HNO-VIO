import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    database_path = os.environ.get("RTABMAP_DB", "/tmp/hno_case009_rtabmap.db")
    odom_frame = os.environ.get("ODOM_FRAME", "odom")
    base_frame = os.environ.get("BASE_FRAME", "base_link")

    stereo_sync = Node(
        package="rtabmap_sync",
        executable="stereo_sync",
        namespace="rtabmap",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": False,
            "approx_sync": True,
            "approx_sync_max_interval": 0.0,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "qos": 1,
            "qos_camera_info": 1,
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
        emulate_tty=True,
        parameters=[{
            "use_sim_time": False,
            "frame_id": base_frame,
            "odom_frame_id": odom_frame,
            "map_frame_id": "map",
            "subscribe_rgbd": True,
            "subscribe_stereo": False,
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "publish_tf": True,
            "publish_tf_map": True,
            "database_path": database_path,
            "wait_for_transform": 1.0,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
            "Reg/Force3DoF": "false",
            "RGBD/OptimizeFromGraphEnd": "true",
            "RGBD/NeighborLinkRefining": "true",
            "Vis/MinInliers": "12",
            "Kp/MaxFeatures": "800",
        }],
        remappings=[
            ("rgbd_image", "rgbd_image"),
        ],
        arguments=["--delete_db_on_start", "--ros-args", "--log-level", "rtabmap.rtabmap:=info"],
    )

    return LaunchDescription([stereo_sync, rtabmap])
