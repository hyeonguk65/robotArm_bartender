import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. arduino_gripper_controller node
    # Controls the custom gripper via Arduino
    gripper_node = Node(
        package='doosan_control',
        executable='arduino_gripper_controller',
        name='arduino_gripper_controller',
        output='screen'
    )

    # 2. main_orchestrator node
    # Manages the main cocktail making sequence
    orchestrator_node = Node(
        package='doosan_control',
        executable='main_orchestrator',
        name='robot_orchestrator',
        output='screen'
    )

    return LaunchDescription([
        gripper_node,
        orchestrator_node
    ])
