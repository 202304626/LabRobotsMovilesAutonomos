from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

import math


def generate_launch_description():
    simulation = False
    dt = 0.05  # sampling period

    # start = (1.0, -1.0, 0.5 * math.pi)  # Outer corridor
    # start = (0.6, -0.6, 1.5 * math.pi)  # Inner corridor

    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[{"simulation": simulation, "dt": dt}],
    )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": (
                    "wall_follower",
                    # "coppeliasim",  # Must be started last
                )
            }
        ],
    )

    # Add the new node to the launch file (5.1)  TO DO
    odometry_node = Node(
        package="amr_turtlebot3",
        executable="odometry_node",
        output="screen",
        # arguments=[],
        # parameters=[]
        
    )

    return LaunchDescription(
        [
            wall_follower_node,
            #coppeliasim_node,
            odometry_node,
            lifecycle_manager_node,  # Must be launched last
        ])