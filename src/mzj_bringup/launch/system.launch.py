"""Simulation is default; hardware mode requires an explicit device."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value("mode")
    if mode not in ("sim", "real"):
        raise ValueError("mode must be sim or real")
    hardware = value("hardware_output").lower() == "true"
    if hardware and mode != "real":
        raise ValueError("hardware_output=true requires mode=real")
    if hardware and not value("serial_device"):
        raise ValueError("hardware output requires serial_device")
    bringup = Path(get_package_share_directory("mzj_bringup"))
    description = Path(get_package_share_directory("mzj_description"))
    config = value("config")
    nodes = []
    def node(package, executable, name, parameters=None, **kwargs):
        item = Node(package=package, executable=executable, name=name,
                    output="screen", parameters=[config] + (parameters or []), **kwargs)
        nodes.append(item)

    node("mzj_planning", "mzj_planning_node", "planning")
    node("mzj_control", "mzj_control_node", "control")
    node("mzj_system", "system_node", "system", [{
        "enabled": mode == "sim", "require_mcu_link": mode == "real"}])
    node("mzj_mcu_bridge", "mcu_bridge", "mcu_bridge", [{
        "dry_run": not hardware, "device": value("serial_device")}])
    node("robot_state_publisher", "robot_state_publisher", "robot_state_publisher", [{
        "robot_description": (description / "urdf" / "parafoil.urdf").read_text()}])
    if mode == "sim":
        node("mzj_simulator", "mzj_simulator_node", "simulator")
        node("mzj_localization", "localization_node", "localization")
    # Real mode requires an external estimator publishing state/odom and its TF.
    if value("perception").lower() == "true":
        node("mzj_perception", "perception_node", "perception", [{
            "backend": value("perception_backend"), "weights": value("weights")}])
        if mode == "sim":
            node("mzj_perception", "demo_camera", "demo_camera")
    if value("rviz").lower() == "true":
        node("rviz2", "rviz2", "rviz2", arguments=[
            "-d", str(bringup / "rviz" / "parafoil.rviz")])
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("mzj_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="sim"),
        DeclareLaunchArgument("config", default_value=str(share / "config" / "system.yaml")),
        DeclareLaunchArgument("hardware_output", default_value="false"),
        DeclareLaunchArgument("serial_device", default_value=""),
        DeclareLaunchArgument("perception", default_value="true"),
        DeclareLaunchArgument("perception_backend", default_value="color_demo"),
        DeclareLaunchArgument("weights", default_value=""),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=build_nodes),
    ])
