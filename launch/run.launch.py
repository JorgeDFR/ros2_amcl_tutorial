import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  dirname, _ = os.path.split(os.path.realpath(__file__))
  params = os.path.join(dirname, 'config.yaml')
  rviz_file = os.path.join(dirname, 'visualization.rviz')
  world_file = os.path.join(dirname, '../webots/worlds/complete_apartment.wbt')
  map_file = os.path.join(dirname, '../webots/maps/complete_apartment.yaml')

  ld = LaunchDescription()

  # Webots Simulator
  webots_simulator = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(dirname, 'webots_sim.launch.py')),
    launch_arguments={'world_file': world_file}.items()
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