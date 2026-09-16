import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'polebot_control'

setup(
    name=package_name,
    version='0.0.1',
    # DIUBAH: find_packages() supaya sub-package pid/, smc/, path_planning/
    # ikut ter-install (sebelumnya cuma [package_name], tidak cukup lagi
    # sejak node-node dipecah ke sub-folder).
    packages=find_packages(exclude=['test']),
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
    description='Controllers (PID/pure-pursuit & SMC) + path planning untuk Polebot AMR',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # --- hardware / CAN (tidak berubah) ---
            'tongyi_rpm_odometry_node = polebot_control.tongyi_rpm_odometry_node:main',
            'tongyi_cmdvel_rpm_logger_node = polebot_control.tongyi_cmdvel_rpm_logger_node:main',
            'cmd_vel_to_tongyi_rpm_node = polebot_control.cmd_vel_to_tongyi_rpm_node:main',
            'hardware_motor_rpm_logger_node = polebot_control.hardware_motor_rpm_logger_node:main',
            'experiment_logger_node = polebot_control.experiment_logger_node:main',

            # --- PID (Kinematic DiffDrive Motion Control) ---
            'pitdt_profiled_pure_pursuit_controller_node = polebot_control.pid.pitdt_profiled_pure_pursuit_controller_node:main',
            'path_profile_node = polebot_control.pid.path_profile_node:main',
            'odom_to_posearray_node = polebot_control.pid.odom_to_posearray_node:main',

            # --- SMC (Dynamic Effort Motion Control) ---
            'sliding_mode_controller = polebot_control.smc.sliding_mode_controller:main',
            'wheel_odom_publisher = polebot_control.smc.wheel_odom_publisher:main',
            'odom_to_tf = polebot_control.smc.odom_to_tf:main',

            # --- Path Planning & Simulation (Modular & Independent) ---
            'bfs_planner = polebot_control.path_planning.bfs_planner:main',
            'trajectory_mode_selector = polebot_control.path_planning.trajectory_mode_selector:main',
            'kinematic_simulator = polebot_control.kinematic_simulator:main',

            # --- Archive Journal (Alipour 2019 reference) ---
            'journal_smc_controller = polebot_control.smc.archive_journal.journal_smc_controller:main',
            'hybrid_smc_controller = polebot_control.smc.archive_journal.hybrid_smc_controller:main',
            'journal_path_planner = polebot_control.smc.archive_journal.journal_path_planner:main',
        ],
    },
)
