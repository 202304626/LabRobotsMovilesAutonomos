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

from std_msgs.msg import String  # Import the standard String message type (std_msgs/String)


class MinimalSubscriber(Node):  # Define a node class that will subscribe to messages

    def __init__(self):  # Constructor: runs when the node object is created
        super().__init__('minimal_subscriber')  # Initialize the Node with the name "minimal_subscriber"
        self.subscription = self.create_subscription(  # Create a subscription (subscriber) and store its handle
            String,  # Message type expected on the topic
            'topic',  # Topic name to subscribe to (must match the publisher's topic)
            self.listener_callback,  # Callback to run every time a message is received (it does not need any timer)
            10)  # Queue depth (QoS history depth) for incoming messages
        self.subscription  # Keep a reference to avoid "unused variable" warnings (and ensure it isn't garbage-collected)

    def listener_callback(self, msg):  # Callback function triggered whenever a message arrives on the topic
        self.get_logger().info('I heard: "%s"' % msg.data)  # Log the received message content (msg.data) to the terminal


def main(args=None):  # Main entry point of the script (optionally receives CLI arguments)
    rclpy.init(args=args)  # Initialize the ROS 2 communication layer for this Python process

    minimal_subscriber = MinimalSubscriber()  # Create an instance of the subscriber node

    rclpy.spin(minimal_subscriber)  # Keep the node alive and process callbacks (here, message reception callbacks)

    # Destroy the node explicitly  # Cleanup: destroy the node object explicitly
    # (optional - otherwise it will be done automatically  # This is optional; Python can clean up automatically
    # when the garbage collector destroys the node object)  # But explicit destruction is clearer and safer
    minimal_subscriber.destroy_node()  # Destroy the node and release its resources
    rclpy.shutdown()  # Shutdown rclpy and cleanly stop ROS 2 communications


if __name__ == '__main__':  
    main()  
