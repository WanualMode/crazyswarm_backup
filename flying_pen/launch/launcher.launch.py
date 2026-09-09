from datetime import datetime
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del context, args, kwargs

    base_bag_dir = os.path.expanduser("~/hitl_ws/src/flying_pen/bag/folder")
    os.makedirs(base_bag_dir, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir = os.path.join(base_bag_dir, f"rosbag2_{ts}")

    fp_share = get_package_share_directory("flying_pen")
    log_player_share = get_package_share_directory("log_player")

    runtime_params = os.path.join(fp_share, "config", "parameters.yaml")
    rviz_visual_params = os.path.join(fp_share, "config", "rviz_visual.yaml")
    urdf_default_path = os.path.join(fp_share, "models", "model.urdf")
    urdf_debug_path = os.path.join(log_player_share, "models", "cf_BLDC.urdf")
    log_player_params = os.path.join(log_player_share, "config", "parameters.yaml")
    wrench_params = os.path.join(log_player_share, "config", "wrench_observer.yaml")
    normal_params = os.path.join(log_player_share, "config", "normal_vector_estimation.yaml")
    shared_su_params = os.path.join(get_package_share_directory("crazyflie"), "config", "su_params.yaml")
    with open(runtime_params, "r", encoding="utf-8") as f:
        runtime_cfg = yaml.safe_load(f) or {}
    with open(shared_su_params, "r", encoding="utf-8") as f:
        shared_su_cfg = yaml.safe_load(f) or {}
    su_wrench_cfg = shared_su_cfg.get("robot_types", {}).get("cf21", {}).get("firmware_params", {}).get("su_wrench", {})
    shared_ee_offset = [
        su_wrench_cfg.get("rOffX", 0.1),
        su_wrench_cfg.get("rOffY", 0.0),
        su_wrench_cfg.get("rOffZ", 0.04),
    ]
    runtime_mode = runtime_cfg.get("runtime", {}).get("ros__parameters", {}).get("mode", "default")
    rviz_config = os.path.join(
        log_player_share,
        "config",
        "log_player_debug.rviz" if runtime_mode == "debug" else "log_player.rviz",
    )
    urdf_path = urdf_debug_path if runtime_mode == "debug" else urdf_default_path

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    actions = []

    actions.append(
        Node(
            package="flying_pen",
            executable="data_logging",
            name="data_logging",
            output="screen",
            parameters=[runtime_params],
        )
    )

    if runtime_mode == "debug":
        actions.append(
            Node(
                package="flying_pen",
                executable="data_logging_debug",
                name="data_logging_debug",
                output="screen",
                parameters=[runtime_params],
            )
        )

    actions.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-o",
                bag_dir,
                "/data_logging_msg",
                *(['/data_logging_msg_debug'] if runtime_mode == "debug" else []),
            ],
            output="screen",
        )
    )

    actions.append(
        TimerAction(
            period=0.5,
            actions=[
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    parameters=[{"robot_description": robot_description}],
                    output="screen",
                )
            ],
        )
    )

    actions.append(
        TimerAction(
            period=0.8,
            actions=[
                Node(
                    package="log_player",
                    executable="log_decoder",
                    name="log_decoder",
                    output="screen",
                    parameters=[log_player_params],
                )
            ],
        )
    )

    actions.append(
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="flying_pen",
                    executable="rviz_visual",
                    name="rviz_visual",
                    output="screen",
                    parameters=[rviz_visual_params, {"end_effector_offset": shared_ee_offset}],
                )
            ],
        )
    )

    actions.append(
        TimerAction(
            period=1.2,
            actions=[
                Node(
                    package="log_player",
                    executable="wrench_observer",
                    name="wrench_observer",
                    output="screen",
                    parameters=[wrench_params],
                )
            ],
        )
    )

    actions.append(
        TimerAction(
            period=1.4,
            actions=[
                Node(
                    package="log_player",
                    executable="normal_vector_estimation",
                    name="normal_vector_estimation",
                    output="screen",
                    parameters=[normal_params],
                )
            ],
        )
    )

    rviz_kwargs = {}
    if os.path.exists(rviz_config):
        rviz_kwargs["arguments"] = ["-d", rviz_config]

    actions.append(
        TimerAction(
            period=1.6,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    **rviz_kwargs,
                )
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
