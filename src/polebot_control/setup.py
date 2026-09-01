import os
from glob import glob
from setuptools import setup

package_name = 'polebot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Controllers for a differential-drive robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tongyi_rpm_odometry_node = polebot_control.tongyi_rpm_odometry_node:main',
            'tongyi_cmdvel_rpm_logger_node = polebot_control.tongyi_cmdvel_rpm_logger_node:main',
            'cmd_vel_to_tongyi_rpm_node = polebot_control.cmd_vel_to_tongyi_rpm_node:main',
            'hardware_motor_rpm_logger_node = polebot_control.hardware_motor_rpm_logger_node:main',
            'pitdt_profiled_pure_pursuit_controller_node = polebot_control.pitdt_profiled_pure_pursuit_controller_node:main',
            'trajectory_node = polebot_control.trajectory_node:main',
            'gain_scheduled_controller_node = polebot_control.gain_scheduled_controller_node:main',
            'path_profile_node = polebot_control.path_profile_node:main',
            'pitdt_pure_pursuit_controller_node = polebot_control.pitdt_pure_pursuit_controller_node:main',
            'odom_to_posearray_node = polebot_control.odom_to_posearray_node:main',
            'experiment_logger_node = polebot_control.experiment_logger_node:main',
        ],
    },
)