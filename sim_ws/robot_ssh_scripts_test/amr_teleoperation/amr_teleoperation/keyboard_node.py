import rclpy
from rclpy.node import Node
from amr_msgs.msg import KeyPressed
from sshkeyboard import listen_keyboard


class KeyPublisher(Node):
    def __init__(self):
        super().__init__("websocket_key")

        self.publisher = self.create_publisher(
            KeyPressed,
            "listen_key",
            10,
        )

        self._running = True

    def on_key_press(self, key):
        msg = KeyPressed()
        msg.key = key
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing key: {msg.key}")


def main(args=None):
    rclpy.init(args=args)

    node = KeyPublisher()

    listen_keyboard(on_press=node.on_key_press)

    # Loop manual ROS
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
