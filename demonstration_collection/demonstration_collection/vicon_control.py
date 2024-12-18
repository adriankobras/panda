import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations
import numpy as np
from vicon_receiver.msg import Position
from std_msgs.msg import String
import math

class RelativeTransformPublisher(Node):
    def __init__(self):
        super().__init__('relative_transform_publisher')

        self.lock_orientation_in_xy_plane = False
        self.start_signal = 'stop'

        # Maybe include check if robot is in home position

        # Define home pose
        # TODO: subscribe to /franka_robot_state_broadcaster/current_pose to get the home pose
        self.ee_home_pose = PoseStamped()
        self.ee_home_pose.pose.position.x = 0.306
        self.ee_home_pose.pose.position.y = 0.0
        self.ee_home_pose.pose.position.z = 0.486
        self.ee_home_pose.pose.orientation.x = 1.0
        self.ee_home_pose.pose.orientation.y = 0.0
        self.ee_home_pose.pose.orientation.z = 0.0
        self.ee_home_pose.pose.orientation.w = 0.0

        # Define position limits
        self.ee_position_limits = {
            'x_min': 0.27, 'x_max': 0.60,
            'y_min': -0.4, 'y_max': 0.4,
            'z_min': -0.0025, 'z_max': 0.6
        }

        # Publisher for relative transform
        self.publisher = self.create_publisher(PoseStamped, 'target_pose', 10)

        # Subscriber to /vicon/cf12/cf12 topic to get position and orientation data
        self.vicon_subscription = self.create_subscription(
            Position,  # Changed to the Position message type
            '/vicon/franka_control/franka_control',
            self.vicon_callback,
            10
        )

        # self.start_signal_subscription = self.create_subscription(
        #     String,
        #     '/start_signal',  # Topic to listen for the start signal
        #     self.signal_callback,
        #     10)

        # Timer to print values every second
        self.print_timer = self.create_timer(0.5, self.print_values)
        # Store the relative position changes for printing
        self.new_ee_position_x = 0.0
        self.new_ee_position_y = 0.0
        self.new_ee_position_z = 0.0
        self.new_ee_orientation = [0.0, 0.0, 0.0, 0.0]
        self.new_ee_orientation_euler = [0.0, 0.0, 0.0]
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = 0.0
        self.delta_orientation = [0.0, 0.0, 0.0, 0.0]
        self.delta_orientation_euler = [0.0, 0.0, 0.0]

        # Store the first reference data (initial transform)
        self.reference_data = None
        self.previous_pose = None

        # Log that the node has been started
        self.get_logger().info('Node initialized, waiting for Vicon data...')

    def vicon_callback(self, msg: Position):
        """
        Callback for receiving new data from the Vicon system.
        This function computes the relative transform compared to the reference.
        """
        if self.start_signal == 'start':
            print('Received start signal! Running the node...')

        if self.reference_data is None:
            # Store the first received data as the reference
            self.reference_data = msg
            self.previous_pose = PoseStamped()
            self.previous_pose.header.frame_id = 'base'
            self.previous_pose.header.stamp = self.get_clock().now().to_msg()
            self.previous_pose.pose = self.ee_home_pose.pose
            self.get_logger().info('Received first data, setting as reference.')
            return

        # Extract reference and new data
        ref_translation = self.reference_data
        new_translation = msg

        # Calculate relative translation (position difference)
        delta_x = new_translation.x_trans - ref_translation.x_trans
        delta_y = new_translation.y_trans - ref_translation.y_trans
        delta_z = new_translation.z_trans - ref_translation.z_trans

        # Calculate the position changes to the home pose
        new_ee_position_x = self.ee_home_pose.pose.position.x + 0.001 * delta_x
        new_ee_position_y = self.ee_home_pose.pose.position.y + 0.001 * delta_y
        new_ee_position_z = self.ee_home_pose.pose.position.z + 0.001 * delta_z

        # Clip the absolute ee position changes to the ee position limits
        new_ee_position_x = self.clip_value(new_ee_position_x, self.ee_position_limits['x_min'], self.ee_position_limits['x_max'])
        new_ee_position_y = self.clip_value(new_ee_position_y, self.ee_position_limits['y_min'], self.ee_position_limits['y_max'])
        new_ee_position_z = self.clip_value(new_ee_position_z, self.ee_position_limits['z_min'], self.ee_position_limits['z_max'])

        # Extract reference and new rotation (quaternion)
        ref_rotation = [ref_translation.x_rot, ref_translation.y_rot, ref_translation.z_rot, ref_translation.w]
        new_rotation = [new_translation.x_rot, new_translation.y_rot, new_translation.z_rot, new_translation.w]

        # Compute the relative rotation using quaternion multiplication
        delta_quaternion = self.compute_relative_quaternion(ref_rotation, new_rotation)

        # Calculate the quaternion changes to the home pose
        ee_home_pose_quaternion = [self.ee_home_pose.pose.orientation.x, self.ee_home_pose.pose.orientation.y, self.ee_home_pose.pose.orientation.z, self.ee_home_pose.pose.orientation.w]
        new_ee_orientation = tf_transformations.quaternion_multiply(delta_quaternion, ee_home_pose_quaternion)

        if self.lock_orientation_in_xy_plane:
            # Lock the orientation in the xy-plane
            new_ee_orientation = [0.0, 0.0, new_ee_orientation[2], new_ee_orientation[3]]

        # Create a TransformStamped message to publish the relative transform
        pose_msg = PoseStamped()

        # Populate the header (frame_id, child_frame_id)
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base'

        # Set the relative translation
        pose_msg.pose.position.x = new_ee_position_x
        pose_msg.pose.position.y = new_ee_position_y
        pose_msg.pose.position.z = new_ee_position_z

        # Set the relative rotation (quaternion)
        pose_msg.pose.orientation.x = new_ee_orientation[0]
        pose_msg.pose.orientation.y = new_ee_orientation[1]
        pose_msg.pose.orientation.z = new_ee_orientation[2]
        pose_msg.pose.orientation.w = new_ee_orientation[3]

        distance_to_previos_pose = np.linalg.norm(np.array([self.previous_pose.pose.position.x, self.previous_pose.pose.position.y, self.previous_pose.pose.position.z]) - np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]))
        if distance_to_previos_pose > 0.10:
            pose_msg = self.previous_pose

        # Publish the relative transform
        self.publisher.publish(pose_msg)

        self.previous_pose = pose_msg

        # Calculate Euler angles for debugging
        delta_euler = self.quaternion_to_euler(delta_quaternion)
        new_euler = self.quaternion_to_euler(new_ee_orientation)

        # Store the relative position changes for printing
        self.new_ee_position_x = new_ee_position_x
        self.new_ee_position_y = new_ee_position_y
        self.new_ee_position_z = new_ee_position_z
        self.new_ee_orientation = new_ee_orientation
        self.new_ee_orientation_euler = new_euler
        self.delta_x = 0.001 * delta_x
        self.delta_y = 0.001 * delta_y
        self.delta_z = 0.001 * delta_z
        self.delta_orientation = delta_quaternion
        self.delta_orientation_euler = delta_euler

        # Log the relative transform for debugging
        # self.get_logger().info(f'\nNew EE Position: ({new_ee_position_x}, {new_ee_position_y}, {new_ee_position_z}), \n'
        #                f'New EE Euler Orientation: ({new_euler[0]}, {new_euler[1]}, {new_euler[2]}), \n'
        #                f'New EE Quaternion Orientation: ({new_ee_orientaion[0]}, {new_ee_orientaion[1]}, {new_ee_orientaion[2]}, {new_ee_orientaion[3]}), \n'
        #                f'Delta Position: ({self.delta_x}, {self.delta_y}, {self.delta_z}), \n'
        #                f'Delta Euler Orientation: ({self.delta_orientation_euler[0]}, {self.delta_orientation_euler[1]}, {self.delta_orientation_euler[2]}), \n'
        #                f'Delta Quaternion Orientation: ({self.delta_orientation[0]}, {self.delta_orientation[1]}, {self.delta_orientation[2]}, {self.delta_orientation[3]})')

    def compute_relative_quaternion(self, q_ref, q_new):
        """
        Compute the relative quaternion from the reference quaternion to the new quaternion.
        This is done by multiplying the new quaternion by the inverse of the reference quaternion.
        """
        # Compute the inverse of the reference quaternion
        q_ref_inv = tf_transformations.quaternion_inverse(q_ref)
        
        # Multiply new quaternion by the inverse of the reference quaternion
        relative_quaternion = tf_transformations.quaternion_multiply(q_new, q_ref_inv)
        
        return relative_quaternion

    def quaternion_to_euler(self, quaternion):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw) using tf_transformations.
        """
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion using tf_transformations.
        """
        return tf_transformations.quaternion_from_euler(roll, pitch, yaw)

    def clip_value(self, value, min_limit, max_limit):
        return max(min(value, max_limit), min_limit)
    
    def print_values(self):
        """
        Print the relative position values every second.
        """
        self.get_logger().info(f'\nNew Position: x: {self.new_ee_position_x:.2f}, y: {self.new_ee_position_y:.2f}, z: {self.new_ee_position_z:.2f}, \n'
               f'Orientation (Euler): x: {self.new_ee_orientation_euler[0]:.2f}, y: {self.new_ee_orientation_euler[1]:.2f}, z: {self.new_ee_orientation_euler[2]:.2f}, \n'
               f'Orientation (Quaternion): x: {self.new_ee_orientation[0]:.2f}, y: {self.new_ee_orientation[1]:.2f}, z: {self.new_ee_orientation[2]:.2f}, w: {self.new_ee_orientation[3]:.2f}, \n'
               f'Delta Position: x: {self.delta_x:.2f}, y: {self.delta_y:.2f}, z: {self.delta_z:.2f}, \n'
               f'Delta Orientation (Euler): x: {self.delta_orientation_euler[0]:.2f}, y: {self.delta_orientation_euler[1]:.2f}, z: {self.delta_orientation_euler[2]:.2f}, \n'
               f'Delta Orientation (Quaternion): x: {self.delta_orientation[0]:.2f}, y: {self.delta_orientation[1]:.2f}, z: {self.delta_orientation[2]:.2f}, w: {self.delta_orientation[3]:.2f} \n')

    def quaternion_to_yaw(x, y, z, w):
        # Calculate yaw (psi) from quaternion in (x, y, z, w) format
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return yaw

    # Example usage
    x, y, z, w = 0, 0, 0.707, 0.707  # Replace with your quaternion values
    yaw_angle = quaternion_to_yaw(x, y, z, w)
    print("Yaw (in radians):", yaw_angle)
    print("Yaw (in degrees):", math.degrees(yaw_angle))



def main(args=None):
    rclpy.init(args=args)
    node = RelativeTransformPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

