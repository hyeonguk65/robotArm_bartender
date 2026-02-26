"""Launch all runtime nodes required by the Gemini VLM workflow."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start perception, gripper service, and robot orchestrator together."""
    return LaunchDescription(
        [
            Node(
                package="gemini_vlm",
                executable="vlm_node",
                name="gemini_vlm_node",
                output="screen",
            ),
        ]
    )
