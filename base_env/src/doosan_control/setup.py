from setuptools import find_packages, setup

package_name = "robotarm"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lewis",
    maintainer_email="yosijki@gmail.com",
    description="Robot Arm Control Package for Bartender Project",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_target_follower = doosan_control.vision_target_lock_follower:main",
            "vision_pick_fsm = doosan_control.vision_pick_and_place_fsm:main",
            "vision_pick_simple = doosan_control.vision_pick_simple:main",
            "main_orchestrator = doosan_control.main_orchestrator:main",
            "menu_store = doosan_control.menu_store:main",
            "rh_p12_map_test = doosan_control.rh_p12_map_and_test:main",
            "arduino_gripper_controller = doosan_control.linux_arduino_gripper_controller:main",
        ],
    },
)
