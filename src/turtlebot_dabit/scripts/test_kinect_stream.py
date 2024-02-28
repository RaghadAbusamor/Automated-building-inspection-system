#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow import keras
from tensorflow.keras.preprocessing import image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.srv import GetMap

class DefectDetectionRobot:
    def init(self):
        # Load the model and class labels
        self.model = keras.models.load_model('/home/raghad/catkin_ws/src/src/kinect_stream/keras_model.h5')
        with open('/home/raghad/catkin_ws/src/src/kinect_stream/labels.txt', 'r') as file:
            self.class_labels = [line.strip() for line in file]

        # Initialize variables for map and camera image
        self.map_data = None
        self.camera_image = None

        # Initialize the camera and map subscribers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Initialize the publisher for robot velocity
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize the action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            self.camera_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Check if map and camera image are available
            if self.map_data is not None and self.camera_image is not None:
                # Process the map and camera image
                self.process_data()

        except Exception as e:
            print(e)

    def map_callback(self, data):
        # Store the received map data
        self.map_data = data

    def process_data(self):
        # Implement image classification and use map information as needed
        # Use self.camera_image for image classification and self.map_data for map information
        # Example: classify images and move forward if a defect is detected

        # Preprocess the frame for prediction
        frame = cv2.resize(self.camera_image, (224, 224))
        img_array = image.img_to_array(frame)
        img_array = np.expand_dims(img_array, axis=0)
        img_array = img_array / 255.0

        # Make a prediction using the loaded model
        predictions = self.model.predict(img_array)

        # Get the class with the highest probability
        predicted_class_index = np.argmax(predictions[0])
        predicted_class = self.class_labels[predicted_class_index]

        # Print the predicted class
        print(f"Predicted class: {predicted_class}")

        # Move the robot if a defect is detected
        if "defect" in predicted_class.lower():
            self.move_robot_forward()

    def move_robot_forward(self):
        # Stop the robot first
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(stop_msg)

        # Move the robot forward by publishing Twist messages
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Adjust the linear velocity as needed
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        # Stop the robot by publishing Twist messages
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    def load_map_from_file(self, map_file):
        # Load map from YAML file
        rospy.wait_for_service('/static_map')
        try:
            get_map = rospy.ServiceProxy('/static_map', GetMap)
            response = get_map(map_file)
            return response.map
        except rospy.ServiceException as e:
            print("Service call failed:", e)

def send_map_to_move_base(self, map_msg):
        # Send the loaded map to move_base
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose.position.x = 0.0
        move_base_goal.target_pose.pose.position.y = 0.0
        move_base_goal.target_pose.pose.orientation.w = 1.0
        move_base_goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base_client.send_goal(move_base_goal)
        self.move_base_client.wait_for_result()
        
if __name__ == 'main':
    rospy.init_node('defect_detection_robot_node')
    defect_detection_robot = DefectDetectionRobot()

    # Example: Load the map from a YAML file and send it to move_base
    map_file_path = '/home/raghad/catkin_ws/gmapping_01.yaml'
    loaded_map = defect_detection_robot.load_map_from_file(map_file_path)
    defect_detection_robot.send_map_to_move_base(loaded_map)

    rospy.spin()