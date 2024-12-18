import os
import rclpy
from geometry_msgs.msg import PoseStamped  # You can replace this with the appropriate message type
from sensor_msgs.msg import Image  # You can replace this with the appropriate message type
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
# from vicon_receiver.msg import Position
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
from pynput import keyboard

class Subscriber(Node):
    def __init__(self):
        super().__init__('image_pose_subscriber')

        """
        Record:
            1. Observation
                1. Camera image (30Hz): 320x240x3
                2. State
                    1. EE pose (x Hz): 3 Position + 1 Angle
                    2. Gripper state (15 Hz): 1 continuous value [0, 1]
            3. Action
                1. Target EE pose (x Hz): 3 Postion + 1 Angle
                2. Gripper command (x Hz): 0 for open, 1 for close

        Final Dataset:
            1. Observation
                    1. Camera image (10Hz): 320x240x3
                    2. State
                        1. EE pose (10Hz): 3 Position + 1 Angle
                        2. Gripper state (10Hz): 1 continuous value [0, 1]
                3. Action
                    1. Target EE pose (10Hz): 3 Postion + 1 Angle
                    2. Gripper command (10Hz): 0 for open, 1 for close

        todo:
            0. Test Quaternion xy-plane locking
            1. Implement correct state subscription: EE pose + Gripper state
                1.1. Implement gripper state subscription: /fr3_gripper/joint_states
            2. Implement correct action subscription: Target EE pose + Gripper command
                2.1. Implement gripper command publishing
            3. Cables
        """
        
        # Subscribers for two topics
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',  # Image data
            self.image_callback,
            10)
        
        # self.image_subscriber_compressed = self.create_subscription(
        #     CompressedImage,
        #     '/camera/camera/color/image_rect_raw/compressed',  # Image data
        #     self.image_callback_compressed,
        #     10)
        
        self.ee_subscriber = self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',
            self.ee_pose_callback,
            10)
        
        self.get_logger().info('Subscriptions initialized.')
        
        # self.gripper_subscriber = self.create_subscription(
        #     JointState,
        #     '/fr3_gripper/joint_states',
        #     self.gripper_state_callback,
        #     10)
        
        # self.vicon_subscriber = self.create_subscription(
        #     Position,
        #     '/vicon/cf12/cf12',  # Replace with your topic name
        #     self.vicon_pose_callback,
        #     10)
        
        self.bag_writer = None
        self.recording = False
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
            type='sensor_msgs/msg/Image',#'sensor_msgs/msg/CompressedImage',
            serialization_format='cdr'
        )
        # topic_info_compressed_image = rosbag2_py.TopicMetadata(
        #     name='/camera/camera/color/image_rect_raw/compressed',
        #     type='sensor_msgs/msg/CompressedImage',
        #     serialization_format='cdr'
        # )
        topic_info_eepose = rosbag2_py.TopicMetadata(
            name='/franka_robot_state_broadcaster/current_pose',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr'
        )
        # topic_info_gripperstate = rosbag2_py.TopicMetadata(
        #     name='/fr3_gripper/joint_states',
        #     type='sensor_msgs/msg/JointState',
        #     serialization_format='cdr'
        # )
        # topic_info_viconpose = rosbag2_py.TopicMetadata(
        #     name='/vicon/cf12/cf12',
        #     type='vicon_receiver/msg/Position',
        #     serialization_format='cdr'
        # )
        self.bag_writer.create_topic(topic_info_image)
        # self.bag_writer.create_topic(topic_info_compressed_image)
        self.bag_writer.create_topic(topic_info_eepose)
        # self.bag_writer.create_topic(topic_info_gripperstate)
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
        self.get_logger().info(f'Image callback triggered: {msg}')
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
        # self.get_logger().info(f'EE Pose callback triggered: {msg}')
        if self.recording:
            self.bag_writer.write(
                '/franka_robot_state_broadcaster/current_pose',
                serialize_message(msg),
                self.get_clock().now().to_msg().sec)
    
    def gripper_state_callback(self, msg):
        width = msg.position[0] + msg.position[1]
        if self.recording:
            self.bag_writer.write(
                '/fr3_gripper/joint_states',
                serialize_message(msg),
                self.get_clock().now().to_msg().sec)
    
    # def vicon_pose_callback(self, msg):
    #     if self.recording:
    #         self.bag_writer.write(
    #             '/vicon/cf12/cf12',
    #             serialize_message(msg),
    #             self.get_clock().now().to_msg().sec)
            

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()

    def toggle_recording(key):
        if key.char == 'x':
            print('Toggling recording')
            if node.recording:
                node.stop_recording()
            else:
                node.start_recording()

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