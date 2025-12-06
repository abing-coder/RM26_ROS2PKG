#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


DEFAULT_MODEL_PATH = "/home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/model/IR_MODEL/new.xml"


def generate_launch_description() -> LaunchDescription:
    container = ComposableNodeContainer(
        name="hik_detection_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="hik_camera",
                plugin="hik_camera::HikCameraNode",
                name="hik_camera",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="recive_pkg",
                plugin="armor_detection::ImageSubscriber",
                name="image_subscriber",
                parameters=[{"model_path": DEFAULT_MODEL_PATH}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
