from setuptools import setup

package_name = "hno_rtabmap_replay"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/replay_case009.launch.py", "launch/rtabmap_case009.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sharpa",
    maintainer_email="sharpa@example.com",
    description="Offline EuRoC stereo replay with saved HNO-VIO odometry for RTAB-Map.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hno_replay_node = hno_rtabmap_replay.replay_node:main",
            "hno_prepare_odom = hno_rtabmap_replay.prepare_odom:main",
            "hno_export_optimized_tf = hno_rtabmap_replay.export_optimized_tf:main",
            "hno_analyze_rtabmap_bag = hno_rtabmap_replay.rtabmap_diagnostics:main",
            "hno_eval_raw_vs_optimized = hno_rtabmap_replay.eval_raw_vs_optimized:main",
        ],
    },
)
