import os
import rclpy
from geometry_msgs.msg import PoseStamped  # You can replace this with the appropriate message type
from sensor_msgs.msg import Image  # You can replace this with the appropriate message type
from sensor_msgs.msg import CompressedImage  # You can replace this with the appropriate message type
# from vicon_receiver.msg import Position
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
from pynput import keyboard

TOGGLE_RECORDING_KEY = 'x'
ABORT_KEY = 'z'

class Subscriber(Node):
    def __init__(self):
        super().__init__('image_pose_subscriber')
        
        # Subscribers for two topics
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',  # Image data
            self.image_callback,
            10)
        
        self.image_subscriber_compressed = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_rect_raw/compressed',  # Image data
            self.image_callback_compressed,
            10)
        
        self.ee_subscriber = self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',  # Replace with your topic name
            self.ee_pose_callback,
            10)
        
        # self.vicon_subscriber = self.create_subscription(
        #     Position,
        #     '/vicon/cf12/cf12',  # Replace with your topic name
        #     self.vicon_pose_callback,
        #     10)
        
        self.bag_writer = None
        self.recording = False
        # self.ready_to_record = False
        self.rosbag_index = 0

        self.get_logger().info('Node started. Press "X" to start/stop recording.')

    def start_recording(self):
        if self.bag_writer:
            self.bag_writer.close()
        
        rosbag_dir = f'rosbags/my_rosbag_{self.rosbag_index}'
        if os.path.exists(rosbag_dir):
            os.system(f'rm -r {rosbag_dir}')
        
        self.bag_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=rosbag_dir, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.bag_writer.open(storage_options, converter_options)
        
        topic_info_image = rosbag2_py.TopicMetadata(
            name='/camera/camera/color/image_rect_raw',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr'
        )
        topic_info_compressed_image = rosbag2_py.TopicMetadata(
            name='/camera/camera/color/image_rect_raw/compressed',
            type='sensor_msgs/msg/CompressedImage',
            serialization_format='cdr'
        )
        topic_info_eepose = rosbag2_py.TopicMetadata(
            name='/franka_robot_state_broadcaster/current_pose',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr'
        )
        # topic_info_viconpose = rosbag2_py.TopicMetadata(
        #     name='/vicon/cf12/cf12',
        #     type='vicon_receiver/msg/Position',
        #     serialization_format='cdr'
        # )
        self.bag_writer.create_topic(topic_info_image)
        self.bag_writer.create_topic(topic_info_compressed_image)
        self.bag_writer.create_topic(topic_info_eepose)
        # self.bag_writer.create_topic(topic_info_viconpose)

        self.get_logger().info(f'Started recording to {rosbag_dir}')
        self.recording = True

    def stop_recording(self):
        if self.bag_writer:
            # self.bag_writer.close()
            self.recording = False
            self.bag_writer = None
            self.get_logger().info('Stopped recording.')
            self.rosbag_index += 1

    def image_callback(self, msg):
        if self.recording:
            self.bag_writer.write(
                '/camera/camera/color/image_rect_raw',
                serialize_message(msg),
                self.get_clock().now().to_msg().sec)
            
    def image_callback_compressed(self, msg):
        if self.recording:
            self.bag_writer.write(
                '/camera/camera/color/image_rect_raw/compressed',
                serialize_message(msg),
                self.get_clock().now().to_msg().sec)

    def ee_pose_callback(self, msg):
        if self.recording:
            self.bag_writer.write(
                '/franka_robot_state_broadcaster/current_pose',
                serialize_message(msg),
                self.get_clock().now().to_msg().sec)
    
    def vicon_pose_callback(self, msg):
        if self.recording:
            self.bag_writer.write(
                '/vicon/cf12/cf12',
                serialize_message(msg),
                self.get_clock().now().to_msg().sec)

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()

    def toggle_recording(key):
        if key.char == TOGGLE_RECORDING_KEY:
            print('Toggling recording')
            if node.recording:
                node.stop_recording()
            else:
                node.start_recording()
        if key.char == ABORT_KEY:
            rclpy.shutdown()
            exit()

    # keyboard.on_press(toggle_recording)
    listener = keyboard.Listener(on_press=toggle_recording)
    listener.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.bag_writer:
            node.bag_writer.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()