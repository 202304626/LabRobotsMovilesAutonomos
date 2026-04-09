import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math

class OdometryPubSub(Node):

    def __init__(self):
        super().__init__('odometry_pub_sub')

        self._prev_x: float | None = None
        self._prev_y: float | None = None
        self._prev_theta: float | None = None
        self._prev_time: float | None = None

        self._odom_publisher = self.create_publisher(Odometry, 'odometry', 10)

        self._odometry_subscriber = self.create_subscription(Odometry,"odom",self.add_vel_odometry,10)

    def add_vel_odometry(self, msg):
        
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract orientation (quaternion -> euler yaw)
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        _, _, theta = quat2euler((qw, qx, qy, qz))

        # Current time in seconds
        t = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        if self._prev_time is not None:
            dt = t - self._prev_time

            if dt > 0.0:
                # Linear velocity: derivative of position
                dx = x - self._prev_x
                dy = y - self._prev_y

                v = math.sqrt(dx * dx + dy * dy) / dt

                # Angular velocity: derivative of orientation
                dtheta = math.atan2(
                    math.sin(theta - self._prev_theta),
                    math.cos(theta - self._prev_theta),
                )
                w = dtheta / dt

                # w = (theta - self._prev_theta) / dt

                # Fill in the twist fields of the original message
                msg.twist.twist.linear.x = float(v)
                msg.twist.twist.linear.y = 0.0
                msg.twist.twist.linear.z = 0.0

                msg.twist.twist.angular.x = 0.0
                msg.twist.twist.angular.y = 0.0
                msg.twist.twist.angular.z = float(w)

                # Publish on /odometry
                self._odom_publisher.publish(msg)

                # self.get_logger().warn(f"Odometry: z_v = {v:.3f} m/s, z_w = {w:.3f} rad/s")

                #self.get_logger().info('I publish v linear: "%s"' % msg.twist.twist.linear.x)
                #self.get_logger().info('I publish w angular: "%s"' % msg.twist.twist.angular.z)

        # Store for next iteration
        self._prev_x = x
        self._prev_y = y
        self._prev_theta = theta
        self._prev_time = t

        


def main(args=None):
    rclpy.init(args=args)

    odometry_pub_sub = OdometryPubSub()

    rclpy.spin(odometry_pub_sub)

    # Destroy the node explicitly
    odometry_pub_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



