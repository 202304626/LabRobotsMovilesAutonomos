#!/usr/bin/env python3
"""
OPTIMIZED Particle Filter Node with KLD-Sampling and HDBSCAN
=============================================================

This is the optimized version of particle_filter_node.py that uses
particle_filter_l.py with KLD-Sampling and HDBSCAN optimizations.

To use this:
1. Install hdbscan: pip install hdbscan
2. Import from particle_filter_l instead of particle_filter
3. Configure KLD parameters in launch file

Author: Claude Sonnet 4.5
Date: 2026-04-17
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

import message_filters
from amr_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import os
import time
import traceback
from transforms3d.euler import euler2quat

# Import particle filter (works when you copy particle_filter_l.py → particle_filter.py)
from amr_localization.particle_filter import ParticleFilter


class ParticleFilterNode(LifecycleNode):
    def __init__(self):
        """Particle filter node initializer with KLD-Sampling support."""
        super().__init__("particle_filter")

        # Standard Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_plot", False)
        self.declare_parameter("global_localization", True)
        self.declare_parameter("initial_pose", (0.0, 0.0, math.radians(0)))
        self.declare_parameter("initial_pose_sigma", (0.05, 0.05, math.radians(5)))
        self.declare_parameter("particles", 1000)
        self.declare_parameter("sigma_v", 0.1)
        self.declare_parameter("sigma_w", 0.1)
        self.declare_parameter("sigma_z", 0.1)
        self.declare_parameter("simulation", False)
        self.declare_parameter("steps_btw_sense_updates", 10)
        self.declare_parameter("world", "lab03")
        self.declare_parameter("min_particles", 100)

        # KLD-Sampling parameters (NEW)
        self.declare_parameter("use_kld_sampling", True)
        self.declare_parameter("kld_epsilon", 0.05)
        self.declare_parameter("kld_delta", 0.01)
        self.declare_parameter("kld_bin_size", 0.2)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """

        try:
            # Standard Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            self._enable_plot = self.get_parameter("enable_plot").get_parameter_value().bool_value
            global_localization = (
                self.get_parameter("global_localization").get_parameter_value().bool_value
            )
            initial_pose = tuple(
                self.get_parameter("initial_pose").get_parameter_value().double_array_value.tolist()
            )
            initial_pose_sigma = tuple(
                self.get_parameter("initial_pose_sigma")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            particles = self.get_parameter("particles").get_parameter_value().integer_value
            sigma_v = self.get_parameter("sigma_v").get_parameter_value().double_value
            sigma_w = self.get_parameter("sigma_w").get_parameter_value().double_value
            sigma_z = self.get_parameter("sigma_z").get_parameter_value().double_value
            self._steps_btw_sense_updates = (
                self.get_parameter("steps_btw_sense_updates").get_parameter_value().integer_value
            )
            self._simulation = self.get_parameter("simulation").get_parameter_value().bool_value
            world = self.get_parameter("world").get_parameter_value().string_value

            # Get min_particles and KLD parameters
            min_particles = self.get_parameter("min_particles").get_parameter_value().integer_value
            use_kld_sampling = self.get_parameter("use_kld_sampling").get_parameter_value().bool_value
            kld_epsilon = self.get_parameter("kld_epsilon").get_parameter_value().double_value
            kld_delta = self.get_parameter("kld_delta").get_parameter_value().double_value
            kld_bin_size = self.get_parameter("kld_bin_size").get_parameter_value().double_value

            # Attribute and object initializations
            self._localized = False
            self._steps = 0
            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )

            # Create OPTIMIZED particle filter with KLD-Sampling
            self._particle_filter = ParticleFilter(
                dt,
                map_path,
                particle_count=particles,
                min_particles=min_particles,
                sigma_v=sigma_v,
                sigma_w=sigma_w,
                sigma_z=sigma_z,
                global_localization=global_localization,
                initial_pose=initial_pose,
                initial_pose_sigma=initial_pose_sigma,
                simulation=self._simulation,
                logger=self.get_logger(),
                # KLD-Sampling parameters
                use_kld_sampling=use_kld_sampling,
                kld_epsilon=kld_epsilon,
                kld_delta=kld_delta,
                kld_bin_size=kld_bin_size,
            )

            if self._enable_plot:
                self._particle_filter.show("Initialization", save_figure=True)

            # Publishers
            self._pose_publisher = self.create_publisher(PoseStamped, "pose", 10)
            self._pose_cov_publisher = self.create_publisher(
                PoseWithCovarianceStamped, "pose_cov", 10
            )

            # Subscribers
            scan_qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            self._subscribers: list[message_filters.Subscriber] = []
            self._subscribers.append(message_filters.Subscriber(self, Odometry, "odometry"))
            self._subscribers.append(
                message_filters.Subscriber(self, LaserScan, "scan", qos_profile=scan_qos_profile)
            )

            ts = message_filters.ApproximateTimeSynchronizer(
                self._subscribers, queue_size=4, slop=1
            )
            ts.registerCallback(self._compute_pose_callback)

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        return super().on_activate(state)

    def _compute_pose_callback(self, odom_msg: Odometry, scan_msg: LaserScan):
        """Subscriber callback. Executes a particle filter and publishes (x, y, theta) estimates.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR sensor readings.

        """
        # Parse measurements
        z_v: float = odom_msg.twist.twist.linear.x
        z_w: float = odom_msg.twist.twist.angular.z
        z_scan: list[float] = scan_msg.ranges

        # Execute particle filter
        self._execute_motion_step(z_v, z_w)
        x_h, y_h, theta_h = self._execute_measurement_step(z_scan)
        self._steps += 1

        # Publish
        self._publish_pose_estimate(x_h, y_h, theta_h)

    def _execute_measurement_step(self, z_scan: list[float]) -> tuple[float, float, float]:
        pose = (float("inf"), float("inf"), float("inf"))
        # Calcular probabilidades y clustering cada N pasos
        if not self._steps % self._steps_btw_sense_updates:
            # Uncomment to measure performance
            # start_time = time.perf_counter()
            self._particle_filter.resample(z_scan)
            # sense_time = time.perf_counter() - start_time
            # self.get_logger().info(f"Sense step time: {sense_time:6.3f} s")

            # start_time = time.perf_counter()
            self._localized, pose = self._particle_filter.compute_pose()
            # clustering_time = time.perf_counter() - start_time
            # self.get_logger().info(f"Clustering time: {clustering_time:6.3f} s")
        else:
            # Vía rápida: actualizar pose usando media si localizado
            if self._localized:
                self._localized, pose = self._particle_filter.compute_pose()

        return pose

    def _execute_motion_step(self, z_v: float, z_w: float):
        """Executes and monitors the motion step (move) of the particle filter.

        Args:
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
        """
        self._particle_filter.move(z_v, z_w)

        if self._enable_plot:
            self._particle_filter.show("Move", save_figure=True)

    def _publish_pose_estimate(self, x_h: float, y_h: float, theta_h: float) -> None:
        """Publishes the robot's pose estimate in a custom amr_msgs.msg.PoseStamped message.

        Args:
            x_h: x coordinate estimate [m].
            y_h: y coordinate estimate [m].
            theta_h: Heading estimate [rad].

        """
        # Create the msg
        msg = PoseStamped()
        msg.localized = self._localized
        msg.header.stamp = self.get_clock().now().to_msg()

        if self._localized:
            msg.pose.position.x = x_h
            msg.pose.position.y = y_h

            w, x, y, z = euler2quat(0.0, 0.0, theta_h)
            msg.pose.orientation.w = w
            msg.pose.orientation.x = x
            msg.pose.orientation.y = y
            msg.pose.orientation.z = z

            # Create PoseWithCovarianceStamped for robot_localization EKF
            msg_cov = PoseWithCovarianceStamped()
            msg_cov.header = msg.header
            msg_cov.header.frame_id = "map"
            msg_cov.pose.pose = msg.pose
            # Assign small covariance since the robot is localized
            msg_cov.pose.covariance[0] = 0.05  # x
            msg_cov.pose.covariance[7] = 0.05  # y
            msg_cov.pose.covariance[35] = 0.05  # yaw
            self._pose_cov_publisher.publish(msg_cov)

        self._pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    particle_filter_node = ParticleFilterNode()

    try:
        rclpy.spin(particle_filter_node)
    except KeyboardInterrupt:
        pass

    particle_filter_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
