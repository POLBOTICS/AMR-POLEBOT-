import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the www directory inside the package's share folder
    www_dir = os.path.join(
        get_package_share_directory('polebot_web_interface'),
        'www'
    )

    # 1. Start the Python HTTP Server on port 8000
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000', '-d', www_dir],
        output='screen'
    )

    # 2. Start rosbridge_websocket on port 9090
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
        }]
    )

    # 3. Start the Web Backend on port 5050 (API & Dashboard)
    web_backend_node = Node(
        package='polebot_web_interface',
        executable='web_backend',
        name='web_backend',
        output='screen'
    )

    # 4. Start rosapi (required for SLAM status check)
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )

    return LaunchDescription([
        http_server,
        rosbridge_node,
        web_backend_node,
        rosapi_node,
    ])
