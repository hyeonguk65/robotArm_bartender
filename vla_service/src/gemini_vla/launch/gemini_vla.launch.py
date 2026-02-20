"""Launch all runtime nodes required by the Gemini VLA workflow."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start perception, gripper service, and robot orchestrator together."""
    return LaunchDescription(
        [
            Node(
                package="gemini_vla",
                executable="vla_node",
                name="gemini_vla_node",
                output="screen",
            ),
            Node(
                package="gemini_vla",
                executable="gripper_service_node",
                name="gripper_service_node",
                namespace="dsr01",
                output="screen",
            ),
            Node(
                package="gemini_vla",
                executable="robot_control",
                name="robot_main_controller",
                namespace="dsr01",
                output="screen",
            ),
        ]
    )
