import os
from datetime import datetime, timezone, timedelta

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 把字符串转成真正的bool值
def as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")

# rosbags-convert 与 rtabmap 要求的格式有差别
def normalize_rosbag2_metadata(bag_path):
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isfile(metadata_path):
        return
    with open(metadata_path, "r", encoding="utf-8") as f:
        text = f.read()
    fixed = text.replace("offered_qos_profiles: []", "offered_qos_profiles: ''")
    if fixed == text:
        return
    with open(metadata_path, "w", encoding="utf-8") as f:
        f.write(fixed)

# 主逻辑函数
def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("hno_vio")
    # 读取 launch 参数
    dataset = LaunchConfiguration("dataset").perform(context)
    config = LaunchConfiguration("config").perform(context)
    bag_path = LaunchConfiguration("bag_path").perform(context)
    play_bag = as_bool(LaunchConfiguration("play_bag").perform(context))
    run_preprocess = as_bool(LaunchConfiguration("run_preprocess").perform(context))
    rviz = as_bool(LaunchConfiguration("rviz").perform(context))
    use_sim_time = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    num_cams = int(LaunchConfiguration("num_cams").perform(context))
    results_root = LaunchConfiguration("results_root").perform(context)
    bag_rate = LaunchConfiguration("bag_rate").perform(context)
    bag_start = LaunchConfiguration("bag_start").perform(context)
    play_topics = LaunchConfiguration("play_topics").perform(context)
    experiment_fix_e_hat = as_bool(
        LaunchConfiguration("experiment_fix_e_hat").perform(context)
    )
    experiment_force_sigma_r_zero = as_bool(
        LaunchConfiguration("experiment_force_sigma_r_zero").perform(context)
    )
    experiment_max_frames = int(
        LaunchConfiguration("experiment_max_frames").perform(context)
    )

    # 自动补全默认路径
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        config_path = os.path.join(pkg_share, "config", config, "estimator_config.yaml")
    camera_config = LaunchConfiguration("camera_config").perform(context)
    if not camera_config:
        camera_config = os.path.join(pkg_share, "config", config, "kalibr_imucam_chain.yaml")
    path_gt = LaunchConfiguration("path_gt").perform(context)
    if not path_gt:
        path_gt = os.path.join(pkg_share, "ground_truth", "euroc_mav", f"{dataset}.txt")

    # 合法性检查
    if run_preprocess and num_cams != 2:
        raise RuntimeError("run_preprocess=true requires num_cams=2 because RTAB-Map input is stereo.")
    if experiment_fix_e_hat and experiment_force_sigma_r_zero:
        raise RuntimeError(
            "experiment_fix_e_hat and experiment_force_sigma_r_zero "
            "cannot both be true."
        )
    if experiment_max_frames < 0:
        raise RuntimeError("experiment_max_frames must be non-negative.")
    if play_bag:
        normalize_rosbag2_metadata(bag_path)

    # 创建本次 run 目录
    beijing = timezone(timedelta(hours=8))
    run_id = datetime.now(beijing).strftime("run_%Y%m%dT%H%M%S")
    run_dir = os.path.join(results_root, run_id)
    vio_results = os.path.join(run_dir, "vio_results")
    os.makedirs(vio_results, exist_ok=True)
    odom_output_path = LaunchConfiguration("odom_output_path").perform(context)
    if not odom_output_path:
        odom_output_path = os.path.join(vio_results, "odom_raw.csv")
    odom_output_path = odom_output_path.replace("{run_id}", run_id)
    rtabmap_input = os.path.join(vio_results, "rtabmap_input_db3")

    # 节点启动区
    actions = []
    ## 启动 HNO-VIO 主节点
    vio_node = Node(
        package="hno_vio",
        executable="run_hno_vio",
        name="run_hno_vio",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "config": config,
            "config_path": config_path,
            "camera_config": camera_config,
            "dataset": dataset,
            "bag_path": bag_path,
            "raw_bag": bag_path,
            "path_gt": path_gt,
            "num_cams": num_cams,
            "use_gt_mapping": as_bool(LaunchConfiguration("use_gt_mapping").perform(context)),
            "try_zupt": as_bool(LaunchConfiguration("try_zupt").perform(context)),
            "update_enforce_structure": as_bool(
                LaunchConfiguration("update_enforce_structure").perform(context)
            ),
            "experiment_fix_e_hat": experiment_fix_e_hat,
            "experiment_force_sigma_r_zero": experiment_force_sigma_r_zero,
            "experiment_max_frames": experiment_max_frames,
            "frontend_print": as_bool(LaunchConfiguration("frontend_print").perform(context)),
            "essential_print": as_bool(LaunchConfiguration("essential_print").perform(context)),
            "updater_print": as_bool(LaunchConfiguration("updater_print").perform(context)),
            "ZUPT_print": as_bool(LaunchConfiguration("ZUPT_print").perform(context)),
            "pipeline_print": as_bool(LaunchConfiguration("pipeline_print").perform(context)),
            "export_odom": as_bool(LaunchConfiguration("export_odom").perform(context)),
            "odom_output_path": odom_output_path,
            "odom_frame": LaunchConfiguration("odom_frame").perform(context),
            "base_frame": LaunchConfiguration("base_frame").perform(context),
            "topic_imu": LaunchConfiguration("topic_imu").perform(context),
            "topic_cam0": LaunchConfiguration("topic_cam0").perform(context),
            "topic_cam1": LaunchConfiguration("topic_cam1").perform(context),
        }],
    )
    actions.append(vio_node)
    if experiment_max_frames > 0:
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=vio_node,
                on_exit=[EmitEvent(event=Shutdown(
                    reason="experiment frame limit reached"
                ))],
            )
        ))

    ## 启动 RTAB-Map 预处理节点
    if run_preprocess:
        actions.append(Node(
            package="hno_vio",
            executable="rtabmap_preprocess",
            name="rtabmap_preprocess",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "camera_config": camera_config,
                "left_topic": LaunchConfiguration("topic_cam0").perform(context),
                "right_topic": LaunchConfiguration("topic_cam1").perform(context),
                "base_frame": LaunchConfiguration("base_frame").perform(context),
                "left_camera_frame": "cam0_rect",
                "right_camera_frame": "cam1_rect",
            }],
        ))
        actions.append(ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "-o", rtabmap_input,
                "/cam0/image_rect",
                "/cam1/image_rect",
                "/cam0/camera_info",
                "/cam1/camera_info",
                "/hno_vio/odom",
                "/tf",
                "/tf_static",
                "/clock",
            ],
            output="screen",
        ))


    ## 启动 rviz 可视化
    if rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(pkg_share, "launch", "hno_vio.rviz")],
            output="screen",
        ))

    ##  自动播放 rosbag
    if play_bag:
        play_cmd = [
            "ros2", "bag", "play", bag_path,
            "--clock", "--rate", bag_rate,
            "--disable-keyboard-controls",
        ]
        if float(bag_start) > 0.0:
            play_cmd.extend(["--start-offset", bag_start])
        if play_topics.strip():
            play_cmd.append("--topics")
            play_cmd.extend(play_topics.split())
        play_process = ExecuteProcess(cmd=play_cmd, output="screen")
        actions.append(TimerAction(period=3.0, actions=[play_process]))
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=play_process,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[EmitEvent(event=Shutdown(reason="input bag playback completed"))],
                    )
                ],
            )
        ))

    return actions

# 参数声明区
def generate_launch_description():
    default_results = "/home/he/hno_vio_ws/src/hno_vio/results"
    return LaunchDescription([
        # 输入数据集参数
        DeclareLaunchArgument("dataset", default_value="V2_02_medium"),
        DeclareLaunchArgument("bag_path", default_value=["/home/he/datasets/euroc/",LaunchConfiguration("dataset"),"_db",],),
        DeclareLaunchArgument("bag_rate", default_value="1.0"),
        DeclareLaunchArgument("bag_start", default_value="0.0"),
        DeclareLaunchArgument("play_topics", default_value="/imu0 /cam0/image_raw /cam1/image_raw"),

        # 配置文件参数
        DeclareLaunchArgument("config", default_value="euroc_mav"),
        DeclareLaunchArgument("config_path", default_value=""),
        DeclareLaunchArgument("camera_config", default_value=""),
        DeclareLaunchArgument("path_gt", default_value=""),
        DeclareLaunchArgument("results_root", default_value=default_results),
        DeclareLaunchArgument("num_cams", default_value="2"),

        # 行为控制
        DeclareLaunchArgument("play_bag", default_value="true"),
        DeclareLaunchArgument("use_gt_mapping", default_value="false"),
        DeclareLaunchArgument("try_zupt", default_value="true"),
        DeclareLaunchArgument("update_enforce_structure", default_value="true"),
        DeclareLaunchArgument("experiment_fix_e_hat", default_value="false"),
        DeclareLaunchArgument("experiment_force_sigma_r_zero", default_value="true"),
        DeclareLaunchArgument("experiment_max_frames", default_value="0"),
        DeclareLaunchArgument("export_odom", default_value="true"),
        DeclareLaunchArgument("run_preprocess", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("odom_output_path", default_value=""),

        # 诊断打印
        DeclareLaunchArgument("essential_print", default_value="true"),
        DeclareLaunchArgument("frontend_print", default_value="false"),
        DeclareLaunchArgument("updater_print", default_value="false"),
        DeclareLaunchArgument("ZUPT_print", default_value="false"),
        DeclareLaunchArgument("pipeline_print", default_value="false"),

        # 话题
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("topic_imu", default_value="/imu0"),
        DeclareLaunchArgument("topic_cam0", default_value="/cam0/image_raw"),
        DeclareLaunchArgument("topic_cam1", default_value="/cam1/image_raw"),

        OpaqueFunction(function=launch_setup),
    ])
