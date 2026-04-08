from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

import math


def generate_launch_description():
    simulation = False
    world = "project"
    start = (-1.0, -1.0, math.radians(90))
    goal = (-0.6, 1.0)
    dt = 0.05  # sampling period

    particles = 50
    global_localization = False
    start_sigma = (0.05, 0.05, math.radians(5))
    sigma_v = 0.1
    sigma_w = 0.25
    sigma_z = 0.15

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
                "global_localization": global_localization,
                "initial_pose": start,
                "initial_pose_sigma": start_sigma,
                "particles": particles,
                "sigma_v": sigma_v,
                "sigma_w": sigma_w,
                "sigma_z": sigma_z,
                "simulation": simulation,
                "world": world,
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
                "connection_distance": 0.3,  # 0.3,
                "enable_plot": True,
                "goal": goal,
                "grid_size": 0.2,
                "node_count": 500,
                "obstacle_safety_distance": 0.15,  # 0.08,
                "simulation": simulation,
                "smoothing_additional_points": 3, #3,
                "smoothing_data_weight": 0.1,
                "smoothing_smooth_weight": 0.25,
                "use_grid": True,
                "world": world,
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
                "lookahead_distance": 0.25,
                "simulation": simulation,
            }
        ],
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
                    "probabilistic_roadmap",
                    "particle_filter",
                    "pure_pursuit",
                )
            }
        ],
    )

    return LaunchDescription(
        [
            odometry_node,
            probabilistic_roadmap_node,
            particle_filter_node,
            pure_pursuit_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )
