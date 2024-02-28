# print Distance to object in front of camera

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
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
            # Get image dimensions
            height, width = depth_image.shape[:2]

            # Get depth value at center of the image
            center_x = width // 2
            center_y = height // 2
            depth_at_center = depth_image[center_y, center_x]

            # Convert depth value from millimeters to meters
            distance_to_object = depth_at_center / 1000.0

            print("Distance to object in front of camera: {:.2f} meters".format(distance_to_object))
        
        rate.sleep()

if __name__ == '__main__':
    main()
