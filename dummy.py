import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import roslibpy

class LocalToRemoteBridge(Node):
    def __init__(self):
        super().__init__('local_to_remote_bridge')
        self.rosbridge_client = roslibpy.Ros(host='localhost', port=8920)
        self.rosbridge_client.run()

        # Define the topics you want to forward
        self.topics = [
            ('/chatter', 'std_msgs/String', '/remote_chatter'),
            ('/another_topic', 'std_msgs/String', '/remote_another_topic'),
            # Add more topics as needed: (local_topic, msg_type, remote_topic)
        ]

        for local_topic, msg_type, remote_topic in self.topics:
            # Create a remote publisher for each topic
            publisher = roslibpy.Topic(self.rosbridge_client, remote_topic, msg_type)

            # Create a local subscriber for each topic
            self.create_subscription(
                String,
                local_topic,
                self.make_callback(publisher, local_topic, remote_topic),
                10)

    def make_callback(self, publisher, local_topic, remote_topic):
        def callback(msg):
            # Forward the message to the remote server via rosbridge
            remote_msg = roslibpy.Message({'data': msg.data})
            publisher.publish(remote_msg)
            #self.get_logger().info(f'Forwarding message from {local_topic} to {remote_topic}: {msg.data}')
        return callback

    def close(self):
        # Clean up the ROSBridge client
        self.rosbridge_client.terminate()

def main(args=None):
    rclpy.init(args=args)
    bridge_node = LocalToRemoteBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
