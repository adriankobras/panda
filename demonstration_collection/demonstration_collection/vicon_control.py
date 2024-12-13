import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations
from vicon_receiver.msg import Position

class RelativeTransformPublisher(Node):
    def __init__(self):
        super().__init__('relative_transform_publisher')

        # Publisher for relative transform
        self.publisher = self.create_publisher(TransformStamped, 'relative_transform', 10)

        # Subscriber to /vicon/cf12/cf12 topic to get position and orientation data
        self.subscription = self.create_subscription(
            Position,  # Changed to the Position message type
            '/vicon/cf12/cf12',
            self.vicon_callback,
            10
        )

        # Store the first reference data (initial transform)
        self.reference_data = None

        # Log that the node has been started
        self.get_logger().info('Node initialized, waiting for Vicon data...')

    def vicon_callback(self, msg: Position):
        """
        Callback for receiving new data from the Vicon system.
        This function computes the relative transform compared to the reference.
        """
        if self.reference_data is None:
            # Store the first received data as the reference
            self.reference_data = msg
            self.get_logger().info('Received first data, setting as reference.')
            return

        # Extract reference and new data
        ref_translation = self.reference_data
        new_translation = msg

        # Calculate relative translation (position difference)
        delta_x = new_translation.x_trans - ref_translation.x_trans
        delta_y = new_translation.y_trans - ref_translation.y_trans
        delta_z = new_translation.z_trans - ref_translation.z_trans

        # Extract reference and new rotation (quaternion)
        ref_rotation = [ref_translation.x_rot, ref_translation.y_rot, ref_translation.z_rot, ref_translation.w]
        new_rotation = [new_translation.x_rot, new_translation.y_rot, new_translation.z_rot, new_translation.w]

        # Compute the relative rotation using quaternion multiplication
        relative_quaternion = self.compute_relative_quaternion(ref_rotation, new_rotation)

        # Create a TransformStamped message to publish the relative transform
        transform_msg = TransformStamped()

        # Populate the header (frame_id, child_frame_id)
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'world'
        transform_msg.child_frame_id = 'cf12_relative'

        # Set the relative translation
        transform_msg.transform.translation.x = delta_x
        transform_msg.transform.translation.y = delta_y
        transform_msg.transform.translation.z = delta_z

        # Set the relative rotation (quaternion)
        transform_msg.transform.rotation.x = relative_quaternion[0]
        transform_msg.transform.rotation.y = relative_quaternion[1]
        transform_msg.transform.rotation.z = relative_quaternion[2]
        transform_msg.transform.rotation.w = relative_quaternion[3]

        # Publish the relative transform
        self.publisher.publish(transform_msg)

        # Calculate Euler angles for debugging
        ref_euler = self.quaternion_to_euler(ref_rotation)
        new_euler = self.quaternion_to_euler(new_rotation)
        delta_roll = new_euler[0] - ref_euler[0]
        delta_pitch = new_euler[1] - ref_euler[1]
        delta_yaw = new_euler[2] - ref_euler[2]

        # Log the relative transform for debugging
        self.get_logger().info(f'Published relative transform: ({delta_x}, {delta_y}, {delta_z}), '
                               f'Rotation: ({delta_roll}, {delta_pitch}, {delta_yaw})')

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

