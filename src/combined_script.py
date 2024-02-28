#!/usr/bin/env python

'''
Combine the image classification and movement scripts to run in parallel
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow import keras
from keras.preprocessing import image
from math import radians

class RobotController():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Load the image classification model
        self.model = keras.models.load_model('/home/raghad/turtlebot2_ws/src/my_kinect_stream/keras_model.h5')
        self.model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

        # Load class labels
        with open('/home/raghad/turtlebot2_ws/src/my_kinect_stream/labels.txt', 'r') as file:
            self.class_labels = [line.strip() for line in file]

        # Initialize CvBridge
        self.bridge = CvBridge()

        # ROS Subscribers
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        rospy.Subscriber('/cmd_vel_mux/input/navi', Twist, self.velocity_callback)

        self.current_position = None  # Store the current robot position

        rospy.spin()

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

            # If the predicted class is not "normal," print the robot position
            if predicted_class != 'normal' and self.current_position is not None:
                rospy.loginfo(f"Detected class: {predicted_class}. Robot position: {self.current_position}")

            # Display the frame with the predicted class name
            cv2.putText(frame, f"{predicted_class}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Processed Image", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def velocity_callback(self, msg):
        # Store the current robot position from the velocity message
        self.current_position = msg.linear.x, msg.linear.y, msg.linear.z

    def shutdown(self):
        rospy.loginfo("Shutting down")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        RobotController()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
