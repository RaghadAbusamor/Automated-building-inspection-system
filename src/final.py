#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow import keras
from keras.preprocessing import image

class GoForwardAndTurn():
    def __init__(self):
        # Initialize action client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

        # Define and send the first goal to move forward by 2 meters
        first_goal = MoveBaseGoal()
        first_goal.target_pose.header.frame_id = 'base_link'
        first_goal.target_pose.header.stamp = rospy.Time.now()
        first_goal.target_pose.pose.position.x = 2
        first_goal.target_pose.pose.orientation.w = 1
        self.move_base.send_goal(first_goal)

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

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

            # Display the frame with the predicted class name
            cv2.putText(frame, f" {predicted_class}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # Show the processed image (optional)
            cv2.imshow("Processed Image", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def shutdown(self):
        rospy.loginfo("Stop")
        # Cancel any active goals
        self.move_base.cancel_goal()

def main():
    try:
        # Initialize the navigation node
        go_forward_and_turn = GoForwardAndTurn()

        # Load the model from the desktop
        go_forward_and_turn.model = keras.models.load_model('/home/raghad/turtlebot2_ws/src/my_kinect_stream/keras_model.h5')

        # Compile the model
        go_forward_and_turn.model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

        # Load the class labels from the desktop
        with open('/home/raghad/turtlebot2_ws/src/my_kinect_stream/labels.txt', 'r') as file:
            go_forward_and_turn.class_labels = [line.strip() for line in file]

        # Initialize the CvBridge
        go_forward_and_turn.bridge = CvBridge()

        # Start the navigation node in a separate thread
        rospy.on_shutdown(go_forward_and_turn.shutdown)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")

if __name__ == "__main__":
    rospy.init_node('nav_test', anonymous=False)
    main()
