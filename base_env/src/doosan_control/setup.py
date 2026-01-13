from setuptools import find_packages, setup

package_name = 'doosan_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'vision_target_follower = doosan_control.vision_target_lock_follower:main',
        'vision_pick_fsm = doosan_control.vision_pick_and_place_fsm:main',
        'vision_pick_simple = doosan_control.vision_pick_simple:main',
        'main_orchestrator = doosan_control.main_orchestrator:main',
        'menu_store = doosan_control.menu_store:main',
    ],
},

)
