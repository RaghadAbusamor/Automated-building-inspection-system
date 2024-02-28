#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Thread
from tensorflow import keras
from keras.preprocessing import image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class ImageClassifierAndNavigator():
    def __init__(self):
        rospy.init_node('image_classifier_and_navigator', anonymous=False)

        # Image classification setup
        self.setup_image_classification()

        # Navigation setup
        self.setup_navigation()

        # ROS Subscriber for the Kinect RGB image
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Throttle the image callback to a maximum frequency
        self.rate = rospy.Rate(5)  # Adjust the frequency as needed

        rospy.spin()

    def setup_image_classification(self):
        # Load the model from the desktop
        self.model = keras.models.load_model('/home/raghad/turtlebot2_ws/src/my_kinect_stream/keras_model.h5')

        # Compile the model (you can adjust the optimizer, loss, and metrics based on your original compilation)
        self.model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

        # Load the class labels from the desktop
        with open('/home/raghad/turtlebot2_ws/src/my_kinect_stream/labels.txt', 'r') as file:
            self.class_labels = [line.strip() for line in file]

        # Initialize the CvBridge
        self.bridge = CvBridge()

    def setup_navigation(self):
        # Navigation setup
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Process the frame in a separate thread for faster predictions
            thread = Thread(target=self.process_image, args=(frame,))
            thread.start()

            # Throttle the callback to control the frequency
            self.rate.sleep()

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def process_image(self, frame):
        try:
            # Preprocess the frame for prediction
            frame = cv2.resize(frame, (224, 224))
            img_array = image.img_to_array(frame)
            img_array = np.expand_dims(img_array, axis=0)
            img_array = img_array / 255.0  # Normalize the image data

            # Make a prediction using the loaded model
            predictions = self.model.predict(img_array)

            # Get the class with the highest probability
            predicted_class_index = np.argmax(predictions[0])
            predicted_class = self.class_labels[predicted_class_index]

            # Print the predicted class
            rospy.loginfo(f"Predicted class: {predicted_class}")

            # Display the frame with the predicted class name (optional)
            cv2.putText(frame, f" {predicted_class}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Processed Image", frame)
            cv2.waitKey(1)

            # After predicting the class and obtaining robot position, initiate navigation
            self.initiate_navigation(predicted_class)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def initiate_navigation(self, predicted_class):
        # Print the predicted class before initiating navigation
        rospy.loginfo(f"Initiating navigation for class: {predicted_class}")

        # Navigation goal setup
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 1.0  # Example goal: move 1 meter forward
        goal.target_pose.pose.orientation.w = 1.0  # Example orientation: go forward

        # Start moving
        self.move_base.send_goal(goal)

        # Allow the robot up to 60 seconds to complete the task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move forward for some reason")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved forward")

        # Print robot position after completing the movement
        self.print_robot_position()

    def print_robot_position(self):
        # Get the robot's current position from the /odom topic
        odom_data = rospy.wait_for_message("/odom", Odometry)
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        rospy.loginfo(f"Current robot position: x = {x}, y = {y}")

    def shutdown(self):
        rospy.loginfo("Stopping image_classifier_and_navigator node")

if __name__ == '__main__':
    try:
        node = ImageClassifierAndNavigator()
        rospy.on_shutdown(node.shutdown)
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")

