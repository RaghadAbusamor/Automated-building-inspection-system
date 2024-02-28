# Coordinates of closest point to the Kinect

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Initialize variables to store image and depth data
depth_image = None
bridge = CvBridge()

def image_callback(data):
    global depth_image
    try:
        # Convert ROS image message to OpenCV image
        depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except Exception as e:
        print(e)

def main():
    rospy.init_node('kinect_depth_node', anonymous=True)
    rospy.Subscriber('/camera/depth/image_raw', Image, image_callback)
    
    rate = rospy.Rate(10)  # Adjust the rate as needed
    
    while not rospy.is_shutdown():
        if depth_image is not None:
            # Filter out depth values below a certain threshold (e.g., 0.1 meters)
            threshold_depth = 0.01  # Adjust threshold as needed
            depth_image_filtered = np.where(depth_image > threshold_depth, depth_image, np.inf)
            
            # Consider a specific region of interest (e.g., excluding edges)
            roi_height, roi_width = depth_image_filtered.shape
            roi_top, roi_bottom = int(0.1 * roi_height), int(0.9 * roi_height)
            roi_left, roi_right = int(0.1 * roi_width), int(0.9 * roi_width)
            depth_roi = depth_image_filtered[roi_top:roi_bottom, roi_left:roi_right]
            
            # Find the coordinates of the closest point to the Kinect
            min_depth = np.min(depth_roi)
            if np.isinf(min_depth):
                print("No valid depth data found")
            else:
                min_depth_coords = np.unravel_index(np.argmin(depth_roi), depth_roi.shape)
                min_depth_x = min_depth_coords[1] + roi_left
                min_depth_y = min_depth_coords[0] + roi_top
                min_distance_to_object = min_depth / 1000.0
                # min_depth_x = min_depth_x/1000.0
                # min_depth_y = min_depth_y/1000.0
                # print (depth_roi)
                # print("Coordinates of closest point to the Kinect - X: {}, Y: {}".format(min_depth_x, min_depth_y))
                print (min_distance_to_object)
        rate.sleep()

if __name__ == '__main__':
    main()
