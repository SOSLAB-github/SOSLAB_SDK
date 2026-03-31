import os
import launch

from launch_ros.actions import Node

def generate_launch_description():

    # Node 1 (Device 0)
    ml_node = Node(
        package="ml",
        executable="ml_node",          # 기존 파일과 동일 :contentReference[oaicite:1]{index=1}
        name="ml_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'lidarType': 'MLX'},
            {'ip_address_device': '192.168.1.10'},
            {'ip_port_device': 2000},
            {'ip_address_pc': '192.168.1.15'},
            {'ip_port_pc': 0}
        ],
    )

    # RViz
    rviz_config_file = os.path.join('../rviz', 'config.rviz')  # 기존 파일 유지 :contentReference[oaicite:2]{index=2}
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_file],
    )

    ld = launch.LaunchDescription()
    ld.add_action(ml_node)
    ld.add_action(rviz_node)

    return ld
