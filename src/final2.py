import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow import keras
from keras.preprocessing import image
from geometry_msgs.msg import Pose, Twist
from math import radians
import paho.mqtt.client as mqtt
import threading
import random

class MoveInCircle():
    def __init__(self):
        # ... (previous code)

        # Initialize current_pose to (0, 0, 0)
        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.0

        # ... (previous code)

        while not rospy.is_shutdown():
            # Move forward for the specified duration
            rospy.loginfo("Moving forward in a circle")
            for _ in range(movement_duration):
                self.cmd_vel.publish(move_cmd)
                self.current_pose.position.x += move_cmd.linear.x  # Update x position as the robot moves
                r.sleep()

            # Turn for the specified duration to create a circular path
            rospy.loginfo("Turning to continue the circle")
            for _ in range(movement_duration):
                self.cmd_vel.publish(turn_cmd)
                self.current_pose.position.z += turn_cmd.angular.z  # Update z position as the robot turns
                r.sleep()

            # Assuming you have a variable current_pose that holds the current pose of the robot
            self.publish_robot_location(self.current_pose)

        # ... (rest of the code)


    def publish_robot_location(self, pose):
        topic = "robot_location"
        payload = f"{pose.position.x},{pose.position.y},{pose.position.z}"
        self.mqtt_client.publish(topic, payload)
        rospy.loginfo(f"Published robot location: {payload}")

    def init_mqtt_client(self):
        client_name_id = "set_here_cliente_id_" + str(random.randint(1, 100)) + "_" + str(random.randint(1, 100))
        mqtt_client = mqtt.Client(client_id=client_name_id, userdata=None, protocol=mqtt.MQTTv31)
        mqtt_client.connect("mqtt-dashboard.com", 1883)
        mqtt_client.loop_start()
        return mqtt_client

    def shutdown(self):
        rospy.loginfo("Stop Moving in Circle")
        self.cmd_vel.publish(Twist())
        self.publish_robot_location(self.current_pose)
        rospy.sleep(1)

def image_callback(msg, move_in_circle):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.resize(frame, (224, 224))
        img_array = image.img_to_array(frame)
        img_array = np.expand_dims(img_array, axis=0)
        img_array = img_array / 255.0

        predictions = model.predict(img_array)
        predicted_class_index = np.argmax(predictions[0])
        predicted_class = class_labels[predicted_class_index]

        print(f"Predicted class: {predicted_class}")

        cv2.putText(frame, f" {predicted_class}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if predicted_class != "normal":
            move_in_circle.publish_robot_location(move_in_circle.current_pose)

        cv2.imshow("Processed Image", frame)
        cv2.waitKey(1)

    except Exception as e:
        print(f"Error processing image: {e}")

def main():
    rospy.init_node('image_classifier_node', anonymous=True)
    move_in_circle = MoveInCircle()

    rospy.Subscriber('/camera/rgb/image_color', Image, image_callback, move_in_circle)

    rospy.spin()

if __name__ == "__main__":
    main()
