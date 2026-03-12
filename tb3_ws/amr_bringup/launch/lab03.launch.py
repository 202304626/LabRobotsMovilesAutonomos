from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

import math


def generate_launch_description():
    simulation = False  # physical robot
    world = "lab03"
    dt = 0.05  # sampling period
    particles = 2000
    sigma_v = 0.05
    sigma_w = 0.1
    sigma_z = 0.2

    particle_filter_node = LifecycleNode(
        package="amr_localization",
        executable="particle_filter",
        name="particle_filter",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "enable_plot": True,
                "global_localization": True,
                "particles": particles,
                "sigma_v": sigma_v,
                "sigma_w": sigma_w,
                "sigma_z": sigma_z,
                "simulation": simulation,
                "world": world,
            }
        ],
    )

    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[{"simulation": simulation, "dt": dt}],
    )

    odometry_node = Node(
        package="amr_turtlebot3",
        executable="odometry_node",
        output="screen",
    )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": (
                    "particle_filter",
                    "wall_follower",
                )
            }
        ],
    )

    return LaunchDescription(
        [
            particle_filter_node,
            wall_follower_node,
            odometry_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )
