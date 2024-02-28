import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class KinectDepthProcessor():
    def __init__(self):
        rospy.init_node('kinect_depth_processor', anonymous=False)

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        rospy.spin()

    def depth_callback(self, msg):
        try:
            # Convert depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Check if the depth image is empty
            if depth_image is None:
                rospy.logwarn("Depth image is empty")
                return

            # Check if the depth image contains valid data
            if np.isnan(depth_image).any():
                rospy.logwarn("Depth image contains NaN values")
                return

            # Scale the depth values to a suitable range (e.g., 0-255)
            depth_image_scaled = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # Process depth data to extract (x, y) point
            # Example: Find the closest point to the camera
            min_depth = np.min(depth_image_scaled)
            min_depth_index = np.unravel_index(np.argmin(depth_image_scaled), depth_image_scaled.shape)

            # Extract (x, y) coordinates from the index
            x = min_depth_index[1]
            y = min_depth_index[0]

            rospy.loginfo(f"Closest point coordinates (x, y): ({x}, {y})")

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

if __name__ == '__main__':
    try:
        node = KinectDepthProcessor()
    except rospy.ROSInterruptException:
        pass
