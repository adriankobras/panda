import os
import rclpy
from geometry_msgs.msg import PoseStamped  # You can replace this with the appropriate message type
from sensor_msgs.msg import Image  # You can replace this with the appropriate message type
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py

class Subscriber(Node):
    def __init__(self):
        super().__init__('image_pose_subscriber')
        
        # Subscribers for two topics
        self.subscriber1 = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',  # Image data
            self.image_callback,
            10)
        
        self.subscriber2 = self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',  # Replace with your topic name
            self.pose_callback,
            10)
        
        # Create a rosbag writer
        self.bag_writer = rosbag2_py.SequentialWriter()
        
        # If it exists, delete the rosbag directory
        if os.path.exists('rosbags/my_rosbag'):
            os.system('rm -r rosbags/my_rosbag')
        # Open the rosbag file
        storage_options = rosbag2_py.StorageOptions(uri='rosbags/my_rosbag', storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.bag_writer.open(storage_options, converter_options)
        
        # Create topics in the rosbag
        topic_info = rosbag2_py.TopicMetadata(
            name='/camera/camera/depth/image_rect_raw',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr'
        )
        self.bag_writer.create_topic(topic_info)
        
        topic_info = rosbag2_py.TopicMetadata(
            name='/franka_robot_state_broadcaster/current_pose',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr'
        )
        self.bag_writer.create_topic(topic_info)

        self.get_logger().info('Node started and rosbag is open.')
    
    def pose_callback(self, msg):
        self.bag_writer.write(
            '/franka_robot_state_broadcaster/current_pose',
            serialize_message(msg),
            self.get_clock().now().to_msg().sec)
    
    def image_callback(self, msg):
        self.bag_writer.write(
            '/camera/camera/depth/image_rect_raw',
            serialize_message(msg),
            self.get_clock().now().to_msg().sec)

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()

    rclpy.spin(node)
    
    # When node is shut down
    node.bag_writer.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
