import rclpy
from rclpy.node import Node
from amr_msgs.msg import KeyPressed
from sshkeyboard import listen_keyboard


class KeyPublisher(Node):
    """
    ROS2 node that listens for keyboard input and publishes pressed keys
    as KeyPressed messages on a topic.

    This node uses the sshkeyboard library to capture key presses from
    the terminal and forwards them into the ROS ecosystem.
    """

    def __init__(self):
        """
        Initializes the KeyPublisher node.

        Creates a ROS publisher that sends KeyPressed messages on the
        'listen_key' topic with a queue size of 10.

        Args:
            None

        Returns:
            None
        """
        # Initialize the ROS2 node with the name "websocket_key"
        super().__init__("websocket_key")

        # Create a publisher for KeyPressed messages on the "listen_key" topic
        self.publisher = self.create_publisher(
            KeyPressed,
            "listen_key",
            10,
        )

        # Internal flag that could be used to control execution
        self._running = True

    def on_key_press(self, key):
        """
        Callback function executed whenever a key is pressed.

        It creates a KeyPressed message, fills it with the pressed key,
        publishes it to the ROS topic, and logs the event.

        Args:
            key (str): The key detected by the keyboard listener.

        Returns:
            None
        """
        # Create a new ROS message
        msg = KeyPressed()

        # Store the pressed key in the message
        msg.key = key

        # Publish the message to the topic
        self.publisher.publish(msg)

        # Print information in the ROS logger
        self.get_logger().info(f"Publishing key: {msg.key}")  


def main(args=None):
    """
    Main entry point for the ROS2 node.

    Initializes ROS, creates the KeyPublisher node, starts the keyboard
    listener, and spins the node manually until ROS is shut down.

    Args:
        args (list, optional): Command-line arguments passed to ROS.

    Returns:
        None
    """
    # Initialize the ROS2 communication layer
    rclpy.init(args=args)

    # Create the node instance
    node = KeyPublisher()

    # Start listening to the keyboard and bind key presses to the callback
    listen_keyboard(on_press=node.on_key_press)

    # Manual ROS loop that processes callbacks while ROS is running
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    # Cleanly destroy the node before exiting
    node.destroy_node()

    # Shut down ROS
    rclpy.shutdown()


if __name__ == "__main__":
    # Run the main function only if the script is executed directly
    main()
