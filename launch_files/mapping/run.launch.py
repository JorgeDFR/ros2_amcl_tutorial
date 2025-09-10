import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
  dirname, _ = os.path.split(os.path.realpath(__file__))
  params = os.path.join(dirname, 'config.yaml')
  rviz_file = os.path.join(dirname, 'visualization.rviz')
  world_file = os.path.join(dirname, '../../stage_sim_worlds/example.world')

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

  # SLAM Toolbox
  slam_toolbox_node = Node(
    package='slam_toolbox',
    executable='sync_slam_toolbox_node',
    name='slam_toolbox',
    parameters=[params],
    output='screen'
  )
  ld.add_action(slam_toolbox_node)

  # Rviz
  rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_file, '--ros-args', '--log-level', 'INFO'],
  )
  ld.add_action(rviz)

  return ld

# In another terminal:
# ros2 run teleop_twist_keyboard teleop_twist_keyboard