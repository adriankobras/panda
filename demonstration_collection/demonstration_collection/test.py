import zarr
import numpy as np
from cv2 import imdecode, IMREAD_COLOR
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from pathlib import Path

def save_to_zarr(output_path, image_msgs, pose_msgs, image_timestamps, pose_timestamps):
    """
    Save collected messages to a Zarr file.
    """
    # Define dataset dimensions and metadata
    n_samples = len(image_msgs)
    image_shape = (96, 96, 3)  # Replace with actual image dimensions
    pose_shape = (7,)  # 3D position + 4D quaternion

    # Open Zarr store
    store = zarr.DirectoryStore(output_path)
    root = zarr.group(store)

    # Create datasets
    images = root.create_dataset("obs/images", shape=(n_samples, *image_shape), dtype="uint8")
    poses = root.create_dataset("obs/poses", shape=(n_samples, *pose_shape), dtype="float32")
    timestamps = root.create_dataset("timestamps", shape=(n_samples,), dtype="float64")

    # Populate datasets
    for i in range(n_samples):
        # Decode image
        image = imdecode(np.frombuffer(image_msgs[i].data, np.uint8), IMREAD_COLOR)
        images[i, :, :, :] = cv2.resize(image, (image_shape[1], image_shape[0]))

        # Get nearest pose
        t = image_timestamps[i]
        pose_idx = np.argmin(np.abs(pose_timestamps - t))
        poses[i, :] = get_pose_from_PoseStamped(pose_msgs[pose_idx])

        # Store timestamp
        timestamps[i] = t

    print(f"Saved dataset to {output_path}.")

def main():
    # Collect and process messages
    image_msgs, pose_msgs = [], []
    image_timestamps, pose_timestamps = [], []

    # Load messages from ROS bag as in your original script
    # (refer to your get_messages method)

    # Save dataset to Zarr
    output_path = str(Path(__file__).parent / "output_dataset.zarr")
    save_to_zarr(output_path, image_msgs, pose_msgs, image_timestamps, pose_timestamps)

if __name__ == "__main__":
    main()

