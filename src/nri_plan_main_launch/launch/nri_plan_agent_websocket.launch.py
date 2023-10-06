from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
             package='nri_plan_agent_websocket',
             namespace='',
             executable='nri_plan_agent_websocket',
             name='nri_plan_agent_websocket_node',
             output='screen',
             parameters=[os.path.join(get_package_share_directory("nri_plan_main_launch"), 'params', 'nri_plan_common.yaml')]
            )
        ]
        )
        
