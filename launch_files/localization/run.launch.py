import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    dirname, filename = os.path.split(os.path.realpath(__file__))
    params = os.path.join(dirname, 'config.yaml')
    rviz_file = os.path.join(dirname, 'visualization.rviz')
    world_file = os.path.join(dirname, '../../stage_sim_worlds/iilab.world')
    map_file = os.path.join(dirname, '../../stage_sim_worlds/maps/iilab.yaml')

    ld = LaunchDescription()

    # Stage Simulator
    stage_simulator_node = Node(
      package='stage_ros',
      executable='stageros2',
      name='stage_ros',
      parameters=[params, {'world': world_file}],
      output='screen'
    )
    ld.add_action(stage_simulator_node)

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