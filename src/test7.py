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
from geometry_msgs.msg import Twist
import paho.mqtt.client as paho
import random
from   paho import mqtt
import time
import random

class ImageClassifierAndNavigator():
    def __init__(self):
        rospy.init_node('image_classifier_and_navigator', anonymous=False)

        # Load the model
        self.model = keras.models.load_model('/home/raghad/turtlebot2_ws/src/my_kinect_stream/keras_model.h5')
        self.model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

        # Load class labels
        with open('/home/raghad/turtlebot2_ws/src/my_kinect_stream/labels.txt', 'r') as file:
            self.class_labels = [line.strip() for line in file]

        self.bridge = CvBridge()
        # self.last_image_time = rospy.Time.now()

    # Minimum time interval between processing images (in seconds)
        # self.min_interval = rospy.Duration(0.1)

        self.client = paho.Client()
        client_name_id = "set_here_client_id_" + str(random.randint(1, 100)) + "_" + str(random.randint(1, 100))
        client = paho.Client(client_id=client_name_id, userdata=None, protocol=paho.MQTTv31)
        client.on_connect = self.on_connect
        client.connect("mqtt-dashboard.com", 1883)
        client.on_subscribe = self.on_subscribe
        client.on_message = self.on_message
        client.on_publish = self.on_publish
        client.subscribe("any_topic_here_to_subscribe/in/our/project", qos=1)

        # self.client.loop_start()  # Start the MQTT client loop

        
  

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    
        # t1 = Thread(target=lambda: client.loop_forever(), args=())
        # t1.start()
        self.setup_navigation()
  
        self.rate = rospy.Rate(2)  

        rospy.spin()

    def setup_navigation(self):
 # Navigation setup
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        # Print a message before initiating navigation
        rospy.loginfo("Initiating navigation")

        twist = Twist()
        twist.linear.x = 0.1  # Decreased linear velocity
        twist.angular.z = 0.0
        
        self.pub.publish(twist)
        # Navigation goal setup
        first_goal = MoveBaseGoal()
        first_goal.target_pose.header.frame_id = 'base_link'
        first_goal.target_pose.header.stamp = rospy.Time.now()
        first_goal.target_pose.pose.position.x = 1.5
        first_goal.target_pose.pose.orientation.w = 1
        self.move_base.send_goal(first_goal)
        
    
        success = self.move_base.wait_for_result(rospy.Duration(60))
        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move forward 2 meters for some reason")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved 2 meters forward")


        # # Define and send the second goal to move left by 1 meter
        second_goal = MoveBaseGoal()
        second_goal.target_pose.header.frame_id = 'base_link'
        second_goal.target_pose.header.stamp = rospy.Time.now()
        second_goal.target_pose.pose.position.y = 2.2
        second_goal.target_pose.pose.orientation.w = 0.5
        self.move_base.send_goal(second_goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move left 2 meter for some reason")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved left 2 meter")

        # Define and send the third goal to move backward by 1 meter
        third_goal = MoveBaseGoal()
        third_goal.target_pose.header.frame_id = 'base_link'
        third_goal.target_pose.header.stamp = rospy.Time.now()
        third_goal.target_pose.pose.position.x = -2
        third_goal.target_pose.pose.orientation.w = 1.0
        self.move_base.send_goal(third_goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move backward 2 meter for some reason")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved backward 2 meter")


  # # Define and send the second goal to move left by 1 meter
        fourth_goal = MoveBaseGoal()
        fourth_goal.target_pose.header.frame_id = 'base_link'
        fourth_goal.target_pose.header.stamp = rospy.Time.now()
        fourth_goal.target_pose.pose.position.y = -1
        fourth_goal.target_pose.pose.orientation.w = 0.5
        self.move_base.send_goal(fourth_goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move left 1 meter for some reason")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved left 1 meter")

    def image_callback(self, msg):
        try:
            # if rospy.Time.now() - self.last_image_time < self.min_interval:
            #     return

            # self.last_image_time = rospy.Time.now()

            roi_x = 150
            roi_y = 150
            roi_width = 224
            roi_height = 224

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

            roi = cv2.resize(roi, (224, 224))
            img_array = image.img_to_array(roi)
            img_array = np.expand_dims(img_array, axis=0)
            img_array = img_array / 255.0

            predictions = self.model.predict(img_array)
            predicted_class_index = np.argmax(predictions[0])
            predicted_class = self.class_labels[predicted_class_index]

            # self.check_class(predicted_class)
            self.print_robot_position(predicted_class)

            cv2.putText(frame, f" {predicted_class}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Processed Image", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def print_robot_position(self, predicted_class):
        odom_data = rospy.wait_for_message("/odom", Odometry)
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        rospy.loginfo(f"Current robot position: x = {x}, y = {y}")
        if "cracks" in predicted_class.lower() or "structural" in predicted_class.lower():
            # rospy.loginfo("predicted class :",predicted_class)
            msg2="0,3,cracks"
            msg = "{},{},{}".format(x, y, predicted_class)
            rospy.loginfo("before publish :",msg)
            self.publish_mqtt("project_topic/automated_building_inspection_system/location", msg2)

            self.publish_mqtt("project_topic/automated_building_inspection_system/location", str(msg))
            rospy.loginfo("after publish :",msg)

    def on_connect(client, userdata, flags, rc, properties=None):
        print("CONNACK received with code %s." % rc)
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    def on_publish(client, userdata,mid, properties=None):
        print("publish OK => mid: " + str(mid))

    def on_subscribe(self,client, userdata, mid, granted_qos, properties=None):
        print("Subscribed Ok => mid: " + str(mid) + " " + str(granted_qos))

    def on_message(self,client, userdata, msg):
        self.subscribe_function(str(msg.topic),str(msg.payload))

    def subscribe_function(topic, msg):
        print("topic:"+topic+"  msg:"+msg)

    def publish_mqtt(self,topic, msg):
        self.client.publish(topic, payload=msg, qos=1)

    def shutdown(self):
        rospy.loginfo("Stopping image_classifier_and_navigator node")
        
if __name__ == '__main__':
    try:
        node = ImageClassifierAndNavigator()
        rospy.on_shutdown(node.shutdown)
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")


