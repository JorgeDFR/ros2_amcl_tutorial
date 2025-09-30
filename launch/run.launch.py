import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  pkg_share = get_package_share_directory('ros2_amcl_tutorial')

  params = os.path.join(pkg_share, 'launch', 'config.yaml')
  rviz_file = os.path.join(pkg_share, 'launch', 'visualization.rviz')
  world_file = os.path.join(pkg_share, 'webots/worlds', 'complete_apartment.wbt')
  map_file = os.path.join(pkg_share, 'webots/maps', 'complete_apartment.yaml')

  ld = LaunchDescription()

  # Webots Simulator
  webots_simulator = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'webots_sim.launch.py')),
    launch_arguments={'world': world_file}.items()
  )
  ld.add_action(webots_simulator)

  # Map server
  map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{'yaml_filename': map_file}],
  )
  ld.add_action(map_server_node)

  # AMCL
  amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    parameters=[params],
  )
  ld.add_action(amcl_node)

  # Lifecycle manager
  lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager',
    output='screen',
    parameters=[{
      'autostart': True,
      'node_names': ['map_server', 'amcl']
    }],
  )
  ld.add_action(lifecycle_manager)

  # Rviz
  rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_file, '--ros-args', '--log-level', 'INFO'],
  )
  ld.add_action(rviz)

  return ld