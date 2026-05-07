#!/usr/bin/env python3
"""
visual_slam_launch.py
Launches RTABMap Visual SLAM for the issac_seven robot.

Usage:
  ros2 launch bugger visual_slam_launch.py
  ros2 launch bugger visual_slam_launch.py localization:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    localization  = LaunchConfiguration('localization').perform(context) == 'true'
    database_path = LaunchConfiguration('database_path').perform(context)

    nodes = []

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'] if not localization else [],
        parameters=[{
            # ── Frames ────────────────────────────────────────────────
            'frame_id':               'base_link',
            'odom_frame_id':          'odom',
            'map_frame_id':           'map',

            # ── Subscriptions ─────────────────────────────────────────
            'subscribe_depth':        False,
            'subscribe_rgb':          False,
            'subscribe_scan':         False,
            'subscribe_odom_info':    False,   # FIXED: was True, /odom_info does not exist
            'subscribe_scan_cloud':   True,

            # ── Sync ──────────────────────────────────────────────────
            'approx_sync':            True,
            'approx_sync_max_interval': 0.1,

            # ── Database ──────────────────────────────────────────────
            'database_path':          database_path,

            # ── Localization vs mapping ────────────────────────────────
            'Mem/IncrementalMemory':  'false' if localization else 'true',
            'Mem/InitWMWithAllNodes': 'true'  if localization else 'false',

            # ── 2D Grid map for Nav2 ──────────────────────────────────
            'Grid/FromDepth':         'false',  # using point cloud, not depth image
            'Grid/3D':                'false',
            'Grid/RayTracing':        'true',
            'Grid/MaxObstacleHeight': '0.5',
            'Grid/CellSize':          '0.05',

            # ── Loop closure ──────────────────────────────────────────
            'Rtabmap/TimeThr':        '0',
            'Rtabmap/DetectionRate':  '1',

            # ── Keyframe thresholds ───────────────────────────────────
            'RGBD/LinearUpdate':      '0.01',
            'RGBD/AngularUpdate':     '0.01',

            # ── Optimisation ──────────────────────────────────────────
            'Optimizer/Strategy':     '1',
            'Optimizer/Slam2D':       'true',

            # ── Output ────────────────────────────────────────────────
            'publish_tf':             True,
            'gen_scan':               False,
            'odom_topic':             '/odom',
        }],
        remappings=[
            ('scan_cloud',  '/camera/points'),
            ('odom',        '/odom'),
            ('grid_map',    '/map'),
        ],
    )
    nodes.append(rtabmap_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='true = localization only, false = mapping'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='Path to RTABMap database file'
        ),
        OpaqueFunction(function=launch_setup),
    ])