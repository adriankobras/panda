import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations
import numpy as np
from vicon_receiver.msg import Position
from signal_waiter import SignalWaiter
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.executors import MultiThreadedExecutor

class RelativeTransformPublisher(Node):
    def __init__(self):
        super().__init__('relative_transform_publisher')

        self.lock_orientation_in_xy_plane = False

        self.processing = False

        self.ee_subscriber = self.create_subscription(
            PoseStamped,
            '/franka_robot_state_broadcaster/current_pose',  # Replace with your topic name
            self.ee_pose_callback,
            10)
        
        # Define home pose for the end-effector (ee)
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
        self.ee_pose = None

        # Debugging
        # self.ee_pose = self.ee_home_pose

        # Log that the node has been started
        self.get_logger().info('Node initialized, waiting for Signal...')

    def ee_pose_callback(self, msg: PoseStamped):
        self.ee_pose = msg

    def toggle_processing(self, signal):
        """
        This method will be called by the SignalWaiter to toggle processing. (added)
        """
        if signal == True:
            self.processing = True
            self.get_logger().info('Started processing.')
        elif signal == False:
            self.processing = False
            self.get_logger().info('Stopped processing.')

    def vicon_callback(self, msg: Position):
        """
        Callback for receiving new data from the Vicon system.
        This function computes the relative transform compared to the reference.
        It only processes data if `self.processing` is True.
        """
        if not self.processing:
            self.reference_data = None
            self.previous_pose = None
            return  # If not processing, do nothing

        if self.reference_data is None:
            # Check if robot is in home pose by calculating the distance of ee pose and ee hom epose
            if self.ee_pose is None:
                self.get_logger().info('No end-effector pose received yet.')
                return
            distance_to_home_pose = np.linalg.norm(np.array([self.ee_pose.pose.position.x, self.ee_pose.pose.position.y, self.ee_pose.pose.position.z]) - np.array([self.ee_home_pose.pose.position.x, self.ee_home_pose.pose.position.y, self.ee_home_pose.pose.position.z]))
            if distance_to_home_pose > 0.02:
                self.get_logger().info(f'Distance to home pose: {distance_to_home_pose}. Robot is not in home pose. Please move the robot to the home pose.')
                return
            
            # Store the first received vicon data as the reference
            self.reference_data = msg

            # Store the home pose as the previous pose initially
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
        if self.lock_orientation_in_xy_plane:
            # Lock the orientation in the xy-plane
            delta_quaternion = [0.0, 0.0, delta_quaternion[2], delta_quaternion[3]]

        # Calculate the quaternion changes to the home pose
        ee_home_pose_quaternion = [self.ee_home_pose.pose.orientation.x, self.ee_home_pose.pose.orientation.y, self.ee_home_pose.pose.orientation.z, self.ee_home_pose.pose.orientation.w]
        new_ee_orientation = tf_transformations.quaternion_multiply(delta_quaternion, ee_home_pose_quaternion)

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
        if self.processing and self.reference_data is not None:
            self.get_logger().info(f'\nNew Position: x: {self.new_ee_position_x:.2f}, y: {self.new_ee_position_y:.2f}, z: {self.new_ee_position_z:.2f}, \n'
               f'Orientation (Euler): x: {self.new_ee_orientation_euler[0]:.2f}, y: {self.new_ee_orientation_euler[1]:.2f}, z: {self.new_ee_orientation_euler[2]:.2f}, \n'
               f'Orientation (Quaternion): x: {self.new_ee_orientation[0]:.2f}, y: {self.new_ee_orientation[1]:.2f}, z: {self.new_ee_orientation[2]:.2f}, w: {self.new_ee_orientation[3]:.2f}, \n'
               f'Delta Position: x: {self.delta_x:.2f}, y: {self.delta_y:.2f}, z: {self.delta_z:.2f}, \n'
               f'Delta Orientation (Euler): x: {self.delta_orientation_euler[0]:.2f}, y: {self.delta_orientation_euler[1]:.2f}, z: {self.delta_orientation_euler[2]:.2f}, \n'
               f'Delta Orientation (Quaternion): x: {self.delta_orientation[0]:.2f}, y: {self.delta_orientation[1]:.2f}, z: {self.delta_orientation[2]:.2f}, w: {self.delta_orientation[3]:.2f} \n')


class SignalWaiter(Node):
    def __init__(self, toggle_callback):
        super().__init__('signal_waiter')

        # Store the toggle callback
        self.toggle_callback = toggle_callback

        # Create a service that accepts a SetBool request
        self.srv = self.create_service(
            SetBool,
            '/signal',
            self.signal_callback
        )

    def signal_callback(self, request, response):
        # Check if the request data is True (for start) or False (for stop)
        if request.data:
            self.toggle_callback(True)  # Call the toggle function to start
            response.success = True
            response.message = 'Start signal received.'
        else:
            self.toggle_callback(False)  # Call the toggle function to stop
            response.success = True
            response.message = 'Stop signal received.'

        return response


def main(args=None):
    rclpy.init(args=args)

    # Create both the main node and the SignalWaiter node
    node = RelativeTransformPublisher()
    signal_waiter = SignalWaiter(node.toggle_processing)

    # Use a multi-threaded executor to handle both nodes concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(signal_waiter)

    try:
        executor.spin()  # Spin both nodes using the executor
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        signal_waiter.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()

