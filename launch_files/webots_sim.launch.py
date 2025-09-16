import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
  dirname, _ = os.path.split(os.path.realpath(__file__))

  package_dir = get_package_share_directory('webots_ros2_turtlebot')

  world = LaunchConfiguration('world',
    default=os.path.join(dirname, '../webots_sim_worlds/iilab.wbt'))
  mode = LaunchConfiguration('mode', default='realtime')
  use_sim_time = LaunchConfiguration('use_sim_time', default=True)

  ld = LaunchDescription()

  # Launch Webots with supervisor enabled
  webots = WebotsLauncher(
    world=PathSubstitution(world),
    mode=mode,
    ros2_supervisor=True
  )
  ld.add_action(webots)
  ld.add_action(webots._supervisor)

  # Robot State Publisher Node
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{
      'robot_description': '<robot name=""><link name=""/></robot>'
    }],
  )
  ld.add_action(robot_state_publisher)

  # Static Transform Publisher Node
  footprint_publisher = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    output='screen',
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
  )
  ld.add_action(footprint_publisher)

  # ROS control spawners
  controller_manager_timeout = ['--controller-manager-timeout', '50']
  controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
  diffdrive_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    output='screen',
    prefix=controller_manager_prefix,
    arguments=['diffdrive_controller'] + controller_manager_timeout,
  )
  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    output='screen',
    prefix=controller_manager_prefix,
    arguments=['joint_state_broadcaster'] + controller_manager_timeout,
  )
  ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

  # TurtleBot Driver Node
  robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
  ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
  mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
  turtlebot_driver = WebotsController(
    robot_name='TurtleBot3Burger',
    parameters=[
      {'robot_description': robot_description_path,
        'use_sim_time': use_sim_time,
        'set_robot_state_publisher': True},
      ros2_control_params
    ],
    remappings=mappings,
    respawn=True
  )
  ld.add_action(turtlebot_driver)

  # Wait for the simulation to be ready to start navigation nodes
  os.environ['TURTLEBOT3_MODEL'] = 'burger'
  waiting_nodes = WaitForControllerConnection(
    target_driver=turtlebot_driver,
    nodes_to_start=ros_control_spawners
  )
  ld.add_action(waiting_nodes)

  # Kill all nodes once the Webots simulation has exited
  kill_nodes_event = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=webots,
      on_exit=[EmitEvent(event=Shutdown())],
    )
  )
  ld.add_action(kill_nodes_event)

  return ld