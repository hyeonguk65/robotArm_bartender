"""Package metadata and console entry points for gemini_vla."""

from setuptools import find_packages, setup

package_name = 'gemini_vla'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gemini_vla.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rms',
    maintainer_email='rms50907932@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # ROS2 entry point names used by launch files.
        'console_scripts': [
            'vla_node = gemini_vla.vla_node:main',
            'robot_control = gemini_vla.robot_control:main',
            'gripper_service_node = gemini_vla.gripper_service_node:main',
        ],
    },
)
