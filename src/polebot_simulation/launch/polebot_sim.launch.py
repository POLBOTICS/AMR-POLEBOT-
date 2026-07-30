"""Gazebo simulation launch - TODO: configure for specific Gazebo version (Fortress/Harmonic)"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # TODO: Add gazebo_ros spawn_entity and world launch here
    ])
