#!/usr/bin/env python3
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
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import traceback

from amr_control.wall_follower import WallFollower

# LiDAR quality of service
qos_lidar_profile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class WallFollowerNode(LifecycleNode):
    def __init__(self):
        """Wall follower node initializer."""
        super().__init__("wall_follower")

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_localization", False)
        self.declare_parameter("simulation", False)
        self._stop_robot = None  # Declare

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        try:
            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            enable_localization = (
                self.get_parameter("enable_localization").get_parameter_value().bool_value
            )
            self._simulation = self.get_parameter("simulation").get_parameter_value().bool_value

            self._stop_robot = False  # Initialize internal variable

            # Attribute and object initializations
            self._wall_follower = WallFollower(
                dt,
                simulation=self._simulation,
                logger=None,
            )

            self._commands_publisher = self.create_publisher(
                msg_type=TwistStamped, topic="cmd_vel", qos_profile=10
            )

            self._subscribers: list[message_filters.Subscriber] = []

            # We append the odometry suscriber
            self._subscribers.append(
                message_filters.Subscriber(self, Odometry, "odometry", qos_profile=10)
            )

            # We append the laser scan suscriber
            self._subscribers.append(
                message_filters.Subscriber(self, LaserScan, "scan", qos_profile=qos_lidar_profile)
            )

            if enable_localization:
                self._subscribers.append(
                    message_filters.Subscriber(self, PoseStamped, "pose", qos_profile=10)
                )

            ts = message_filters.ApproximateTimeSynchronizer(
                self._subscribers,
                queue_size=10,
                slop=10,
            )

            ts.registerCallback(self._compute_commands_callback)

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

    def _compute_commands_callback(
        self,
        odom_msg: Odometry,
        scan_msg: LaserScan,
        pose_msg: PoseStamped = PoseStamped(),
    ):
        """Subscriber callback. Executes a wall-following controller and publishes v and w commands.

        Ceases to operate once the robot is localized.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR readings.
            pose_msg: Message containing the estimated robot pose.

        """
        if not pose_msg.localized:
            if self._stop_robot:
                self._publish_velocity_commands(0.0, 0.0)
            else:
                z_v: float = odom_msg.twist.twist.linear.x
                z_w: float = odom_msg.twist.twist.angular.z

                z_scan: list[float] = list(scan_msg.ranges)

                v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)
                self._publish_velocity_commands(v, w)

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """Publishes velocity commands in a geometry_msgs.msg.TwistStamped message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].

        """

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = v
        msg.twist.angular.z = w

        self._commands_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()

    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
