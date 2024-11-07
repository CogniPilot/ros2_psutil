from pathlib import Path

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, \
        DeclareLaunchArgument, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    arg_update_frequency = DeclareLaunchArgument(
        'update_frequency',
        default_value='1.0',
        description='Update frequency to publish in Hz.'
    )
    arg_individual_topics = DeclareLaunchArgument(
        'individual_topics',
        default_value='False',
        description='Publish individual topics instead of unified psutil topic.'
    )
    arg_mem = DeclareLaunchArgument(
        'mem',
        default_value='True',
        description='Publish memory.'
    )
    arg_net = DeclareLaunchArgument(
        'net',
        default_value='True',
        description='Publish network.'
    )
    arg_net_addr = DeclareLaunchArgument(
        'net_addr',
        default_value='True',
        description='Publish network address.'
    )
    arg_net_state = DeclareLaunchArgument(
        'net_state',
        default_value='True',
        description='Publish network state.'
    )
    arg_net_stats = DeclareLaunchArgument(
        'net_stats',
        default_value='True',
        description='Publish network stats.'
    )
    arg_temp = DeclareLaunchArgument(
        'temp',
        default_value='True',
        description='Publish temperature.'
    )
    arg_proc = DeclareLaunchArgument(
        'proc',
        default_value='True',
        description='Publish processor.'
    )
    arg_proc_per = DeclareLaunchArgument(
        'proc_per',
        default_value='True',
        description='Publish processor frequency.'
    )
    arg_proc_freq = DeclareLaunchArgument(
        'proc_freq',
        default_value='True',
        description='Publish processor percent.'
    )
    arg_net_nic_match = DeclareLaunchArgument(
        'net_nic_match',
        default_value='[""]',
        description='Array of network NICs to return, to return all leave empty.'
    )
    arg_net_af_match = DeclareLaunchArgument(
        'net_af_match',
        default_value='[-9999]',
        description='''Int array of network address families to return, valid values are: 
            AF_INET: 2, 
            AF_INET6: 10, 
            AF_PACKET: 17, 
            to return all leave empty.'''
    )
    arg_dev_temp_match = DeclareLaunchArgument(
        'dev_temp_match',
        default_value='[""]',
        description='Array of devices to return temperatures for, to return all leave empty.'
    )
    arg_temp_name_match = DeclareLaunchArgument(
        'temp_name_match',
        default_value='[""]',
        description='Array of temperture names on a device to return, to return all leave empty.'
    )
    arg_log_level = DeclareLaunchArgument(
        'log_level',
        default_value=['warn'],
        description='Logging level'
    )

    node_ros2_psutil = Node(
       package='ros2_psutil',
       output='screen',
       executable='psutil_node',
       arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
       parameters=[
         {'update_frequency': LaunchConfiguration('update_frequency')},
         {'individual_topics': LaunchConfiguration('individual_topics')},
         {'mem': LaunchConfiguration('mem')},
         {'net': LaunchConfiguration('net')},
         {'net_addr': LaunchConfiguration('net_addr')},
         {'net_state': LaunchConfiguration('net_state')},
         {'net_stats': LaunchConfiguration('net_stats')},
         {'temp': LaunchConfiguration('temp')},
         {'proc': LaunchConfiguration('proc')},
         {'proc_per': LaunchConfiguration('proc_per')},
         {'proc_freq': LaunchConfiguration('proc_freq')},
         {'net_nic_match': LaunchConfiguration('net_nic_match')},
         {'net_af_match': LaunchConfiguration('net_af_match')},
         {'dev_temp_match': LaunchConfiguration('dev_temp_match')},
         {'temp_name_match': LaunchConfiguration('temp_name_match')},
       ],
       remappings=[],
    )

    return LaunchDescription([
        arg_update_frequency,
        arg_individual_topics,
        arg_mem,
        arg_net,
        arg_net_addr,
        arg_net_state,
        arg_net_stats,
        arg_temp,
        arg_proc,
        arg_proc_per,
        arg_proc_freq,
        arg_net_nic_match,
        arg_net_af_match,
        arg_dev_temp_match,
        arg_temp_name_match,
        arg_log_level,
        node_ros2_psutil
    ])
