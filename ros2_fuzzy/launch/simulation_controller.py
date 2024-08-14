import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
   
    return LaunchDescription([
       
       
    IncludeLaunchDescription(
    	XMLLaunchDescriptionSource(
    	os.path.join(get_package_share_directory('rosbridge_server'),'launch/rosbridge_websocket_launch.xml')
        )
    ),
    
   Node(
          
            package='ros2_fuzzy',
            executable='spawner',
            output = 'screen',
            respawn=True,
            respawn_delay=2
           ),
])

