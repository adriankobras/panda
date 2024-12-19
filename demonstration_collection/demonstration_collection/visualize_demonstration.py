import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
# from vicon_receiver.msg import Position
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
import rosbag2_py
import cv2
import time
import sys
import os
import pathlib
# from utils.numpy_to_zarr import numpy_to_zarr

ROOT_DIR = str(pathlib.Path(__file__).parent.parent.parent)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

def get_time_from_msg(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

def get_pose_from_PoseStamped(msg):
    return np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                     msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

def get_pose_from_Position(msg):
    return np.array([msg.x_trans, msg.y_trans, msg.z_trans, 
                     msg.x_rot, msg.y_rot, msg.z_rot, msg.w])

class ImageVisualizer(Node):
    def __init__(self):
        super().__init__('image_visualizer')
        
        # Initialize the CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Initialize rosbag reader
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=ROOT_DIR + '/demonstration_collection/rosbags/my_rosbag_0', storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)
        
        # Variables to control image visualization
        self.image_count = 0
        self.ee_pose_count = 0
        # self.vicon_pose_count = 0
        
        # Variable to store the messages
        self.image_msgs = []
        self.image_msgs_timestamps = []
        self.ee_pose_msgs = []
        self.ee_pose_msgs_timestamps = []
        # self.vicon_pose_msgs = []
        # self.vicon_pose_msgs_timestamps = []
        
    def get_messages(self):
        while self.reader.has_next():
            topic, msg, t = self.reader.read_next()
            if topic == '/camera/camera/color/image_rect_raw':
                if isinstance(msg, bytes):
                    msg = deserialize_message(msg, Image)
                self.image_msgs.append(msg)
                self.image_msgs_timestamps.append(get_time_from_msg(msg))
                self.image_count += 1
            if topic == '/camera/camera/color/image_rect_raw/compressed':
                if isinstance(msg, bytes):
                    msg = deserialize_message(msg, CompressedImage)
                self.image_msgs.append(msg)
                self.image_msgs_timestamps.append(get_time_from_msg(msg))
                self.image_count += 1
            elif topic == '/franka_robot_state_broadcaster/current_pose':
                if isinstance(msg, bytes):
                    msg = deserialize_message(msg, PoseStamped)
                self.ee_pose_msgs.append(msg)
                self.ee_pose_msgs_timestamps.append(get_time_from_msg(msg))
                self.ee_pose_count += 1
            # elif topic == '/vicon/cf12/cf12':
            #     if isinstance(msg, bytes):
            #         msg = deserialize_message(msg, Position)
            #     self.vicon_pose_msgs.append(msg)
            #     self.vicon_pose_msgs_timestamps.append(msg.frame_number)
            #     self.vicon_pose_count += 1

        self.image_msgs_timestamps = np.array(self.image_msgs_timestamps)
        self.ee_pose_msgs_timestamps = np.array(self.ee_pose_msgs_timestamps)
        # self.vicon_pose_msgs_timestamps = np.array(self.vicon_pose_msgs_timestamps)
        self.get_logger().info(f'Collected {self.image_count} images, {self.ee_pose_count} ee poses.')

    def create_dataset(self):
        """Function to create a dataset from the collected messages"""
        # Create a dataset from the collected messages
        self.dataset = []
        if not self.image_msgs:
            self.get_logger().error('No image messages collected.')
            return
        
        start_time = self.image_msgs[0].header.stamp.sec + self.image_msgs[0].header.stamp.nanosec * 1e-9
        for i in range(self.image_count):
            # image = self.bridge.imgmsg_to_cv2(self.image_msgs[i], desired_encoding='bgr8') # Image
            image = cv2.imdecode(np.frombuffer(self.image_msgs[i].data, np.uint8), cv2.IMREAD_COLOR) # CompressedImage
            t = self.image_msgs[i].header.stamp.sec + self.image_msgs[i].header.stamp.nanosec * 1e-9 
            
            # Get ee_pose and vicon_pose messages based on the timestamp
            ee_pose_idx = np.argmin(np.abs(self.ee_pose_msgs_timestamps - t))
            # vicon_pose_idx = np.argmin(np.abs(self.vicon_pose_msgs_timestamps - t))

            ee_pose = get_pose_from_PoseStamped(self.ee_pose_msgs[ee_pose_idx])
            # vicon_pose = get_pose_from_Position(self.vicon_pose_msgs[vicon_pose_idx])

            self.dataset.append((image, ee_pose, t - start_time)) # 30Hz?
        
        # Save the dataset to a file
        np.save(ROOT_DIR + '/demonstration_collection/dataset.npy', self.dataset)
        print(f"Dataset created with {len(self.dataset)} samples.")

        # Save the dataset to a zarr file
        # numpy_to_zarr(self.dataset, ROOT_DIR + '/demonstration_collection/dataset.zarr')

    def visualize_images(self):
        """Function to visualize the images in the dataset"""
        for sample in self.dataset:
            print(f"Time: {sample[2]}")

            print(isinstance(sample[0], np.ndarray))
            # Display the image
            cv2.imshow('Image', sample[0])
            cv2.waitKey(1)  # Wait for 1 ms to display the image
            
            # Optional: pause between images to see them clearly
            time.sleep(0.05)

        # Wait for any key press to close all images
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ImageVisualizer()
    
    # Read all messages from the rosbag
    node.get_messages()    

    # Create a dataset from the collected messages
    node.create_dataset()

    # Visualize the images
    node.visualize_images()

    node.destroy_node()

    # Shutdown the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
