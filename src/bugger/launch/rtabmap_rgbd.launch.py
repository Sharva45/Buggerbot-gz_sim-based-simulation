from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,      # flip True once you fuse with lidar
        'approx_sync': True,
        'use_sim_time': True,
        'qos': 2,
        'Reg/Force3DoF': 'true',      # ground robot, not a drone — lock roll/pitch/z
    }]

    remappings = [
        ('rgb/image', '/camera/image'),
        ('depth/image', '/camera/depth_image'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('odom', '/odom'),
    ]

    return LaunchDescription([
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters, remappings=remappings),
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters, remappings=remappings,
            arguments=['-d']),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters, remappings=remappings),
    ])