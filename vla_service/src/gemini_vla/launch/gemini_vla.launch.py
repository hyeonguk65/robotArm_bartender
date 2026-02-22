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
        ]
    )
