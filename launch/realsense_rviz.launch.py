from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('hello_moveit'), 'config', 'realsense_rviz.rviz'])


    ## include realsense2_camera
    launch_realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            # 'pose_fps' : '15',
            # 'rgb_camera.profile': '1280x720x30'
        }.items(),
    )

    node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
        )

    return LaunchDescription([launch_realsense2_camera, node])
