# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy  # Import the ROS 2 Python client library (rclpy)
from rclpy.node import Node  # Import the base Node class to create ROS 2 nodes

from std_msgs.msg import (
    String,
)  # Import the standard String message type (std_msgs/String) to use it to pass data on the topic


class MinimalPublisher(
    Node
):  # Define a node class that will publish messages, it inherits from the class Node (Node is the parent class)
    def __init__(self):  # Constructor: runs when the node object is created
        super().__init__(
            "minimal_publisher"
        )  # Initialize the Node with the name "minimal_publisher", usign the parent's constructor

        self.publisher_ = self.create_publisher(
            String, "topic", 10
        )  # Create a publisher on the topic called 'topic' with queue size of 10 (queue is a quality of service)
        # It has to be a method defined in the parent class: Node

        timer_period = 0.5  # seconds  # Set the timer period (publish every 0.5 s)
        self.timer = self.create_timer(
            timer_period, self.timer_callback
        )  # Create a timer that calls the function timer_callback periodically (every timer_period secs)
        # It has to be a method defined in the parent class: Node

        self.i = 0  # Initialize a counter used to change the message content each time

    def timer_callback(
        self,
    ):  # Callback function executed every time the timer triggers, because we created the self.timer on the __init__
        msg = String()  # Create a new String message object
        msg.data = (
            "Hello World: %d" % self.i
        )  # Fill the string message payload with text + the current counter value (a decimal value, the self.i value)
        self.publisher_.publish(
            msg
        )  # Publish the message to the configured topic (already specified when created the publisher itself)

        self.get_logger().info(
            'Publishing: "%s"' % msg.data
        )  # Print an info log to the terminal with the sent data
        self.i += 1  # Increment the counter for the next timer call


def main(args=None):  # Main entry point of the script (optionally receives CLI arguments)
    rclpy.init(args=args)  # Initialize the ROS 2 communication layer for this Python process

    minimal_publisher = (
        MinimalPublisher()
    )  # Create an instance of the publisher node, using the class MinimalPublisher()

    rclpy.spin(
        minimal_publisher
    )  # Keep the node alive and process callbacks (e.g., the timer callback)
    # This makes the node keep listening to callbacks

    # Destroy the node explicitly  # Cleanup: destroy the node object explicitly
    # (optional - otherwise it will be done automatically  # This is optional; Python can clean up automatically
    # when the garbage collector destroys the node object)  # But explicit destruction is clearer and safer
    minimal_publisher.destroy_node()  # Destroy the node and release its resources
    # It has to be a method defined in the parent class: Node

    rclpy.shutdown()  # Shutdown rclpy and cleanly stop ROS 2 communications


if __name__ == "__main__":
    main()
