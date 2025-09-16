"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import roslibpy

class LocalToRemoteBridge(Node):
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)
        self.rosbridge_client = roslibpy.Ros(host='localhost', port=9090)
        self.rosbridge_client.run()

        # Local ROS2 subscriber
        self.local_subscriber = self.create_subscription(
            String,
            '/mono_py_driver/img_msg',
            self.local_callback,
            10)

        # Remote rosbridge publisher
        self.remote_publisher = roslibpy.Topic(self.rosbridge_client, topic_name, 'sensor_msgs/Image') #sensor_msgs::msg::Image

    def local_callback(self, msg):
        # Forward the message to the remote server via rosbridge
        print('alma')
        remote_msg = roslibpy.Message({'data': msg.data})
        self.remote_publisher.publish(remote_msg)
        self.get_logger().info(f'Forwarding message: {msg.data}')

    def close(self):
        # Clean up resources
        self.remote_publisher.unsubscribe()
        self.rosbridge_client.terminate()

def main(args=None):
    rclpy.init(args=args)
    bridge_node = LocalToRemoteBridge('node_name','/topic_name')
    #bridge_node2 = LocalToRemoteBridge('local_to_remote_bridge','/remote_chatter2')
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""


import base64
import roslibpy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self, ros_client, remote_topic):
        super().__init__('image_subscriber')
        self.ros_client = ros_client
        self.remote_topic = remote_topic
        self.subscription = self.create_subscription(
            Image,
            '/mono_py_driver/img_msg',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to CV2 image format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Encode image as jpeg
        _, buffer = cv2.imencode('.jpg', cv_image)
        jpg_as_text = np.array(buffer).tobytes()

        # Encode image data as base64 to ensure JSON compatibility
        jpg_as_base64 = base64.b64encode(jpg_as_text).decode('utf-8')

        # Publish image data to remote server via roslibpy
        if self.ros_client.is_connected:
            self.remote_topic.publish(roslibpy.Message({'data': jpg_as_base64}))
            self.get_logger().info('Image data forwarded to remote server as base64.')

def main(args=None):
    rclpy.init(args=args)

    # Setup WebSocket connection to the remote server
    client = roslibpy.Ros(host='localhost', port=9090)  # Update with your remote host details
    client.run()

    # Setup topic for publishing to remote server
    remote_topic = roslibpy.Topic(client, '/remote_image_topic', 'std_msgs/String')

    image_subscriber = ImageSubscriber(client, remote_topic)

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        image_subscriber.destroy_node()
        rclpy.shutdown()
        remote_topic.unadvertise()
        client.terminate()

if __name__ == '__main__':
    main()
