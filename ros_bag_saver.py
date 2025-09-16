import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def extract_images_from_bag(bag_path, color_topic, depth_topic, output_dir):
    bridge = CvBridge()

    os.makedirs(os.path.join(output_dir, "color"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "depth"), exist_ok=True)

    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    # Map topic names to types
    type_map = {t.name: t.type for t in topic_types}

    color_count = 0
    depth_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = type_map[topic]

        if msg_type == "sensor_msgs/msg/Image":
            msg = deserialize_message(data, Image)

            if topic == color_topic:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                filename = os.path.join(output_dir, "color", f"color_{color_count:05d}.png")
                rotated_color = cv2.flip(cv_img, -1)
                cv2.imwrite(filename, rotated_color)
                color_count += 1

            elif topic == depth_topic:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                filename = os.path.join(output_dir, "depth", f"depth_{depth_count:05d}.png")
                rotated_depth = cv2.flip(cv_img, -1)
                cv2.imwrite(filename, rotated_depth)
                depth_count += 1

    print(f"Saved {color_count} color images and {depth_count} depth images to '{output_dir}'")

# Example usage:
extract_images_from_bag("/rosbags", "/camera/color/image_raw", "/camera/depth/image_raw", "./pole_recording")
