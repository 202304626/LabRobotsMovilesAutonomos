"""
OPTIMIZED Launch File with KLD-Sampling and HDBSCAN
====================================================

This launch file uses the optimized particle filter with:
- KLD-Sampling: Adaptive particle count
- HDBSCAN: Fast clustering (10-50x speedup)

To use:
1. Install hdbscan: pip install hdbscan
2. Run: ros2 launch amr_bringup project_l.launch.py

Author: Claude Sonnet 4.5
Date: 2026-04-17
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import math


def generate_launch_description():
    simulation = True
    world = "project"
    start = (-1, 0.6, math.radians(90))
    goal = (-0.6, 1.0)

    # OPTIMIZED Particle Filter with KLD-Sampling and HDBSCAN
    particle_filter_node = LifecycleNode(
        package="amr_localization",
        executable="particle_filter",  # Still uses same executable
        name="particle_filter",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],  # Set to INFO to see KLD logs
        parameters=[
            {
                "dt": 0.05,
                "enable_plot": False,
                "global_localization": True,
                "particles": 3000,  # Maximum initial particles
                "min_particles": 200,  # Minimum when localized (KLD will adjust)
                "sigma_v": 0.05,
                "sigma_w": 0.1,
                "sigma_z": 0.2,
                "simulation": simulation,
                "world": world,
                # KLD-Sampling configuration
                "use_kld_sampling": True,  # Enable adaptive particle count
                "kld_epsilon": 0.05,  # Max error (smaller = more particles)
                "kld_delta": 0.01,  # Confidence 99% (1 - 0.01)
                "kld_bin_size": 0.2,  # Spatial discretization [m]
            }
        ],
    )

    probabilistic_roadmap_node = LifecycleNode(
        package="amr_planning",
        executable="probabilistic_roadmap",
        name="probabilistic_roadmap",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "connection_distance": 0.30,
                "enable_plot": False,
                "goal": goal,
                "grid_size": 0.2,
                "node_count": 800,
                "obstacle_safety_distance": 0.12,
                "simulation": simulation,
                "smoothing_additional_points": 10,
                "smoothing_data_weight": 0.1,
                "smoothing_smooth_weight": 0.1,
                "use_grid": True,
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
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "dt": 0.05,
                "enable_localization": True,
                "simulation": simulation,
            }
        ],
    )

    pure_pursuit_node = LifecycleNode(
        package="amr_control",
        executable="pure_pursuit",
        name="pure_pursuit",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "dt": 0.05,
                "lookahead_distance": 0.20,
                "simulation": simulation,
            }
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[PathJoinSubstitution([FindPackageShare("amr_bringup"), "config", "ekf.yaml"])],
    )

    coppeliasim_node = LifecycleNode(
        package="amr_simulation",
        executable="coppeliasim",
        name="coppeliasim",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "dt": 0.05,
                "enable_localization": True,
                "goal": goal,
                "pose_tolerance": (0.1, 10.0),
                "start": start,
            }
        ],
    )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": (
                    "probabilistic_roadmap",
                    "wall_follower",
                    "particle_filter",
                    "pure_pursuit",
                    "coppeliasim",  # Must be started last
                )
            }
        ],
    )

    return LaunchDescription(
        [
            particle_filter_node,
            probabilistic_roadmap_node,
            wall_follower_node,
            pure_pursuit_node,
            ekf_node,
            coppeliasim_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )
