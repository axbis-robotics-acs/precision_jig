from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            launch_arguments=[
                ('rgb_camera.color_profile', '1280,720,30'), #max 1920*1080*30 rgb
                ('align_depth.enable', 'true'),
            ]
        ),
        
        Node(
            package='precision_jig',
            executable='amr_vision_check_gui',
            name='amr_vision_check_gui',
            output='screen',
        ),

    ])