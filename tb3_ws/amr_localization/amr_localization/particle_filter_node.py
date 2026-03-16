import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

import message_filters
from amr_msgs.msg import PoseStamped, ControlStop
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import os
import time
import traceback
from transforms3d.euler import euler2quat

from amr_localization.particle_filter import ParticleFilter


class ParticleFilterNode(LifecycleNode):
    def __init__(self):
        """Particle filter node initializer."""
        super().__init__("particle_filter")

        # Parameters
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
        
        # 3.11.3 Declare parameter for saving history
        self._odometry_estimate_list = []
        self._scan_last_measures = []

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(
            f"Transitioning from '{state.label}' to 'inactive' state."
        )

        try:

            self._odometry_estimate_list = []

            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            self._enable_plot = (
                self.get_parameter("enable_plot").get_parameter_value().bool_value
            )
            global_localization = (
                self.get_parameter("global_localization")
                .get_parameter_value()
                .bool_value
            )
            initial_pose = tuple(
                self.get_parameter("initial_pose")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            initial_pose_sigma = tuple(
                self.get_parameter("initial_pose_sigma")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            particles = (
                self.get_parameter("particles").get_parameter_value().integer_value
            )
            sigma_v = self.get_parameter("sigma_v").get_parameter_value().double_value
            sigma_w = self.get_parameter("sigma_w").get_parameter_value().double_value
            sigma_z = self.get_parameter("sigma_z").get_parameter_value().double_value
            self._steps_btw_sense_updates = (
                self.get_parameter("steps_btw_sense_updates")
                .get_parameter_value()
                .integer_value
            )
            self._simulation = (
                self.get_parameter("simulation").get_parameter_value().bool_value
            )
            world = self.get_parameter("world").get_parameter_value().string_value

            # Attribute and object initializations
            self._localized = False
            self._steps = 0
            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )
            self._particle_filter = ParticleFilter(
                dt,
                map_path,
                particle_count=particles,
                sigma_v=sigma_v,
                sigma_w=sigma_w,
                sigma_z=sigma_z,
                global_localization=global_localization,
                initial_pose=initial_pose,
                initial_pose_sigma=initial_pose_sigma,
                simulation=self._simulation,
                logger=self.get_logger(),  # Replace None with self.get_logger() to enable logging in the class
            )

            if self._enable_plot:
                self._particle_filter.show("Initialization", save_figure=True)

            # Publishers
            # TODO: 3.1. Create the /pose publisher (PoseStamped message).
            self._pose_publisher = self.create_publisher(PoseStamped, "pose", 10)

            # Subscribers
            scan_qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            self._scan_subscriber = self.create_subscription(
                LaserScan, "scan", self._callback_scan_saving_history, scan_qos_profile
            )

            self._odometry_subscriber = self.create_subscription(
                Odometry, "odometry", self._callback_odometry_saving_history, 10
            )

            # 3.11.2 Create publisher for the stop condition
            self._stop_publisher = self.create_publisher(
                ControlStop, "stop_condition", 10
            )

            timer = self.create_timer(5, self._timer_callback)



        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)
    
    def _timer_callback(self):
        """Timer callback to check the stop condition and publish it."""

        # a)
        stop_msg = ControlStop()
        stop_msg.stop_robot = True  # We set the stop condition to true if the robot is localized
        stop_msg.header.stamp = self.get_clock().now().to_msg()  # We add the header (stamp)
        self._stop_publisher.publish(stop_msg)  # We publish the stop
        
        # b)
        odometry_estimates = list(self._odometry_estimate_list)

        self._odometry_estimate_list.clear()
        for z_v, z_w in odometry_estimates:
            self._execute_motion_step(z_v, z_w)
            self._steps += 1
        self.get_logger().info(f"b) {self._steps}")

        # c)
        x_h, y_h, theta_h = 0.0, 0.0, 0.0
        if self._scan_last_measures:
           x_h, y_h, theta_h = self._execute_measurement_step(self._scan_last_measures)
           self._scan_last_measures = []

        # d)
        stop_msg = ControlStop()
        stop_msg.stop_robot = self._localized  # We set the stop condition to true if the robot is localized
        stop_msg.header.stamp = self.get_clock().now().to_msg()  # We add the header (stamp)
        self._stop_publisher.publish(stop_msg)  # We publish the stop

        # e)
        if self._localized:
            x_h, y_h, theta_h = self._particle_filter.compute_pose()[1]
            self._publish_pose_estimate(x_h, y_h, theta_h)
            

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

    def _execute_measurement_step(
        self, z_scan: list[float]
    ) -> tuple[float, float, float]:
        """Executes and monitors the measurement step (sense) of the particle filter.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].

        Returns:
            Pose estimate (x_h, y_h, theta_h) [m, m, rad]; inf if cannot be computed.
        """
        pose = (float("inf"), float("inf"), float("inf"))

        start_time = time.perf_counter()

        self._particle_filter.resample(z_scan)
        sense_time = time.perf_counter() - start_time

        if self._enable_plot:
            self._particle_filter.show("Sense", save_figure=True, display = True)

        start_time = time.perf_counter()
        self._localized, pose = self._particle_filter.compute_pose()
        clustering_time = time.perf_counter() - start_time

        self.get_logger().info(f"Clustering time: {clustering_time:6.3f} s")

        return pose

    def _execute_motion_step(self, z_v: float, z_w: float):
        """Executes and monitors the motion step (move) of the particle filter.

        Args:
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
        """
        start_time = time.perf_counter()
        self._particle_filter.move(z_v, z_w)
        move_time = time.perf_counter() - start_time

    def _publish_pose_estimate(self, x_h: float, y_h: float, theta_h: float) -> None:
        """Publishes the robot's pose estimate in a custom amr_msgs.msg.PoseStamped message.

        Args:
            x_h: x coordinate estimate [m].
            y_h: y coordinate estimate [m].
            theta_h: Heading estimate [rad].

        """
        # TODO: 3.2. Complete the function body with your code (i.e., replace the pass statement).

        # Create the msg
        msg = PoseStamped()
        msg.localized = (
            self._localized
        )  # We add this information wether is true or false
        msg.header.stamp = self.get_clock().now().to_msg()  # We add the header (stamp)

        if (
            self._localized
        ):  # If the robot is localized, we add the pose info to the msg
            msg.pose.position.x = x_h
            msg.pose.position.y = y_h

            w, x, y, z = euler2quat(
                0.0, 0.0, theta_h
            )  # Roll and pitch are zero. We introduce yaw
            msg.pose.orientation.w = w
            msg.pose.orientation.x = x
            msg.pose.orientation.y = y
            msg.pose.orientation.z = z

        self._pose_publisher.publish(msg)  # We publish the msg
    

    # 3.11.3 Callback functions for saving history of /scan and /odometry subscriptions
    def _callback_scan_saving_history(self,scan_msg: LaserScan):
        """Subscriber callback. Executes a particle filter and publishes (x, y, theta) estimates.

        Args:
            scan_msg: Message containing LiDAR sensor readings.

        """
        # Parse measurements
        z_scan: list[float] = scan_msg.ranges

        self._scan_last_measures = z_scan
    
    # 3.11.3 /odometry
    def _callback_odometry_saving_history(self, odom_msg: Odometry):
        """Subscriber callback. Executes a particle filter and publishes (x, y, theta) estimates.

        Args:
            odom_msg: Message containing odometry measurements.

        """
        # Parse measurements
        z_v: float = odom_msg.twist.twist.linear.x
        z_w: float = odom_msg.twist.twist.angular.z

        if abs(z_v) >= 1e-6 or abs(z_w) >= 1e-6:
            self._odometry_estimate_list.append((z_v, z_w))

        
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
