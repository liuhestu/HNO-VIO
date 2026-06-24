import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def env_default(name, value):
    return os.environ.get(name, value)


def generate_launch_description():
    euroc_mav0 = DeclareLaunchArgument(
        "euroc_mav0",
        default_value=env_default("EUROC_MAV0", "/home/sharpa/datasets/euroc/ASL/V1_01_easy/mav0"),
    )
    odom_csv = DeclareLaunchArgument(
        "odom_csv",
        default_value=env_default(
            "HNO_VIO_ODOM_CSV",
            "/home/sharpa/hno_vio_clean/src/hno_vio/success_odom/case009_guarded_020/offline_results/hno_vio_odom.csv",
        ),
    )
    max_duration_sec = DeclareLaunchArgument("max_duration_sec", default_value=env_default("MAX_DURATION_SEC", "0.0"))
    max_odom_time_diff_sec = DeclareLaunchArgument("max_odom_time_diff_sec", default_value="0.03")
    replay_rate_hz = DeclareLaunchArgument("replay_rate_hz", default_value="20.0")

    replay = Node(
        package="hno_rtabmap_replay",
        executable="hno_replay_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "euroc_mav0": LaunchConfiguration("euroc_mav0"),
            "odom_csv": LaunchConfiguration("odom_csv"),
            "max_odom_time_diff_sec": LaunchConfiguration("max_odom_time_diff_sec"),
            "replay_rate_hz": LaunchConfiguration("replay_rate_hz"),
            "max_duration_sec": LaunchConfiguration("max_duration_sec"),
            "odom_frame": "odom",
            "base_frame": "base_link",
            "left_camera_frame": "cam0_rect",
            "right_camera_frame": "cam1_rect",
        }],
    )
    return LaunchDescription([euroc_mav0, odom_csv, max_duration_sec, max_odom_time_diff_sec, replay_rate_hz, replay])
