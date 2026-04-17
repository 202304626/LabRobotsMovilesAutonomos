import math
import rclpy
from rclpy.node import Node
import numpy as np
from amr_msgs.msg import KeyPressed
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)



# Maximum allowed angular velocity (rad/s)
MAX_ANGULAR_VEL = 2.81

# Maximum allowed linear velocity (m/s)
MAX_LINEAR_VEL = 0.22

# Distance in meters considered dangerous for obstacles
OBSTACLE_THRESHOLD = 0.25

# QoS profile for LiDAR sensor data:
# BEST_EFFORT is common for sensors because occasional drops are acceptable.
qos_lidar_profile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# QoS profile for velocity commands:
# RELIABLE ensures commands are delivered.
qos_cmdvel_profile = QoSProfile(
    history=QoSHistoryPolicy.UNKNOWN,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class NodeMapper(Node):
    """
    ROS2 node that converts keyboard input into velocity commands and
    stops the robot if an obstacle is detected using LiDAR data.

    The node subscribes to:
        - 'listen_key' (KeyPressed): keyboard commands.
        - 'scan' (LaserScan): LiDAR ranges.

    And publishes:
        - 'cmd_vel' (Twist): velocity commands for the robot.
    """

    def __init__(self) -> None:
        """
        Initializes the NodeMapper node.

        Creates publishers and subscribers, defines the keyboard mapping,
        and initializes velocity state variables.

        Args:
            None

        Returns:
            None
        """
        # Initialize the ROS2 node with name "node_mapper"
        super().__init__("node_mapper")

        self.obstacle_stop_front = False
        self.obstacle_stop_back = False

        # Mapping of keyboard keys to linear and angular velocity increments
        # Format: key -> (linear_increment, angular_increment)
        self.map = {
            "w": (1.0, 0.0),
            "s": (-1.0, 0.0),
            "a": (0.0, -1.0),
            "d": (0.0, 1.0),
            "space": (0.0, 0.0),
        }

        # Current velocity state
        self.vel_lin = 0.0
        self.vel_ang = 0.0

        # Publisher for robot velocity commands
        self._publisher = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        # Subscriber for keyboard messages
        self._subscriber = self.create_subscription(
            msg_type=KeyPressed,
            topic="listen_key",
            callback=self.key_callback,
            qos_profile=10,
        )

        # Subscriber for LiDAR scan messages
        self.teleoperationLiDARSubs = self.create_subscription(
            msg_type=LaserScan,
            topic="scan",
            callback=self.AnalyzeLiDAR,
            qos_profile=qos_lidar_profile,
        )

    def AnalyzeLiDAR(self, msg: LaserScan):

        N = len(msg.ranges)
        if N == 0 or msg.angle_increment == 0.0:
            self.get_logger().warn("Empty scan or angle_increment=0")
            return

        def min_range_around(target_angle_rad: float, half_window_deg: float = 10.0) -> float:
            half_window = math.radians(half_window_deg)

            i_center = int((target_angle_rad - msg.angle_min) / msg.angle_increment)
            i_delta = max(1, int(half_window / msg.angle_increment))

            i0 = max(0, i_center - i_delta)
            i1 = min(N - 1, i_center + i_delta)

            best = float("inf")
            for i in range(i0, i1 + 1):
                r = msg.ranges[i]
                if math.isfinite(r) and (msg.range_min <= r <= msg.range_max):
                    if r < best:
                        best = r
            return best

        # Distances
        dist_front = min_range_around(0.0, half_window_deg=10.0)
        dist_back  = min_range_around(math.pi, half_window_deg=10.0)

        self.distance_front = dist_front
        self.distance_back = dist_back

        self.get_logger().info(f"Front: {dist_front:.3f} m | Back: {dist_back:.3f} m")

        danger_front = dist_front < OBSTACLE_THRESHOLD
        danger_back  = dist_back  < OBSTACLE_THRESHOLD

        self.get_logger().info(f"Angular vel: {self.vel_ang}")
        self.get_logger().info(f"Linear vel: {self.vel_lin}")

        if danger_front or danger_back:

            if not self.obstacle_stop_front and not self.obstacle_stop_back:

                self.vel_lin = 0.0
                self.vel_ang = 0.0

                stop_msg = Twist()
                stop_msg.linear.x = self.vel_lin
                stop_msg.angular.z = self.vel_ang
                self._publisher.publish(stop_msg)

            if danger_front:
                self.obstacle_stop_front = True

            else:
                self.obstacle_stop_back = True

        else:
            self.obstacle_stop_front, self.obstacle_stop_back = False, False


    def key_callback(self, msg: KeyPressed) -> None:
        """
        Callback executed when a KeyPressed message is received.

        Extracts the pressed key and forwards it to the key processing logic.

        Args:
            msg (KeyPressed): Incoming keyboard message.

        Returns:
            None
        """
        key = msg.key

        if not self.obstacle_stop_back and key == "s":
            self.process_key(key)

        elif not self.obstacle_stop_front and key == "w":
            self.process_key(key)

        elif key not in ["s", "w"]:
            self.process_key(key)


        


    def process_key(self, key):
        """
        Processes a keyboard key and updates the robot velocity.

        Applies incremental changes to the linear and angular velocities,
        enforces maximum limits, and publishes Twist commands.

        Args:
            key (str): Pressed keyboard key.

        Returns:
            None
        """
        if key in self.map:

            # Immediate stop command
            if key == "space":

                # No velocity to stop
                self.vel_lin = 0.0
                self.vel_ang = 0.0

                self.get_logger().info("Stopping...")

                # Publish velocity command
                t = Twist()
                t.linear.x = self.vel_lin
                t.angular.z = self.vel_ang

                self._publisher.publish(t)

            else:
                lin, ang = self.map.get(key)

                # Increment velocities
                self.vel_lin += lin * 0.05
                self.vel_ang += ang * 0.3

                # Safety checks for velocity limits
                if abs(self.vel_ang) >= MAX_ANGULAR_VEL:
                    self.get_logger().info("Max angular velocity reached")

                elif abs(self.vel_lin) >= MAX_LINEAR_VEL:
                    self.get_logger().info("Max linear velocity reached")

                else:
                    # Publish velocity command
                    t = Twist()
                    t.linear.x = self.vel_lin
                    t.angular.z = self.vel_ang

                    self.get_logger().info(f"Transmiting key: {key}")
                    self._publisher.publish(t)
 
        else:
            self.get_logger().info(f"Not a valid key: {key}")


def main(args=None) -> None:
    """
    Entry point for the NodeMapper ROS2 node.

    Initializes ROS2, creates the node, spins it to process callbacks,
    and performs cleanup on shutdown.

    Args:
        args (list, optional): Command-line arguments passed to ROS2.

    Returns:
        None
    """
    # Initialize ROS2 
    rclpy.init(args=args)

    # Create the node instance
    mapping_node = NodeMapper()

    # Keep the node alive and processing callbacks
    rclpy.spin(mapping_node)

    # Explicitly destroy the node before exit
    mapping_node.destroy_node()

    # Shutdown ROS2
    rclpy.shutdown()


# Script entry point
if __name__ == "__main__":
    main()
