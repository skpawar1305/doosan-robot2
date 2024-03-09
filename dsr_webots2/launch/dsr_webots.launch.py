#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)

from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node

package_name = "dsr_webots2"
package_dir = get_package_share_directory(package_name)
xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')

ARGUMENTS = [
    DeclareLaunchArgument(
        'model',
        default_value='m1013',
        description='Robot Model'
    ),
    DeclareLaunchArgument(
        'color',
        default_value='white',
        description='Robot Color'
    )
]	


# Define all the ROS 2 nodes that need to be restart on simulation reset here
def get_ros2_nodes(context, *args):
    robot_desc = Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])
    robot_desc = robot_desc.perform(context)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": True,
            }
        ],
    )

    # For webots, dae mesh files are found only if there's no "file:", so we remove it
    robot_desc = robot_desc.replace("file:", "")
    robot_desc = robot_desc.replace("////", "/")

    # Webots doesn't load up the one .dae file
    robot_desc = robot_desc.replace(
        "dsr_description2/share/dsr_description2/meshes/m1013_white/MF1013_2_1.dae",
        f"{package_name}/share/{package_name}/worlds/MF1013_2_1.obj"
    )
    spawn_URDF_dsr = URDFSpawner(
        name='dsr',
        robot_description=robot_desc,
    )

    ros2_control_params = os.path.join(
        package_dir, "config", "ros2_controllers.yaml"
    )

    # Driver node
    driver = WebotsController(
        robot_name="dsr",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "config", "ros2_control.urdf"
                )
            },
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            ros2_control_params,
        ],
    )

    # ROS2 control spawners for dsr
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )

    ros2_control_spawners = [
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=driver, nodes_to_start=ros2_control_spawners
    )

    return [spawn_URDF_dsr, driver, waiting_nodes, robot_state_publisher]


def generate_launch_description():
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "dsr.wbt"])
    )
    ros2_supervisor = Ros2SupervisorLauncher()

    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=OpaqueFunction(function = get_ros2_nodes),
        )
    )

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription(
        ARGUMENTS +
        [
            webots,
            ros2_supervisor,
            webots_event_handler,
            reset_handler,
            OpaqueFunction(function = get_ros2_nodes),
        ]
    )
