import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'polebot_research_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mirae',
    maintainer_email='mirae@polebot.local',
    description='PoleBot Research Motion Controllers and Path Planners',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pitdt_profiled_pure_pursuit_controller_node = polebot_research_control.pid.pitdt_profiled_pure_pursuit_controller_node:main',
            'odom_to_posearray_node = polebot_research_control.pid.odom_to_posearray_node:main',
            'path_profile_node = polebot_research_control.pid.path_profile_node:main',
            'sliding_mode_controller = polebot_research_control.smc.sliding_mode_controller:main',
            'wheel_odom_publisher = polebot_research_control.smc.wheel_odom_publisher:main',
            'odom_to_tf = polebot_research_control.smc.odom_to_tf:main',
            'bfs_planner = polebot_research_control.path_planning.bfs_planner:main',
            'trajectory_mode_selector = polebot_research_control.path_planning.trajectory_mode_selector:main',
        ],
    },
)
