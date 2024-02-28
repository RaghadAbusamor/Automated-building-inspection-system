#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
import paho.mqtt.client as mqtt

class GoForwardAvoid():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Set up the action client to communicate with the move_base server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

        # Set up MQTT client
        self.mqtt_client = mqtt.Client()
       # Replace with your MQTT broker address
        self.mqtt_client.connect("mqtt-dashboard.com", 1883)


        # Create a goal to move 3 meters forward
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 2.0  # 3 meters
        goal.target_pose.pose.orientation.w = 1.0  # go forward

        # Send the goal to the robot
        self.move_base.send_goal(goal)

        # Wait for the robot to complete the task within 60 seconds
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move forward 3 meters for some reason")
        else:
            # The robot successfully moved forward
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved 3 meters forward")

                # Get the robot's current location
                current_pose = self.move_base.get_result().pose

                # Publish robot location to MQTT
                self.publish_robot_location("project_topic/automated_building_inspection_system/location",current_pose)

    def publish_robot_location(self, pose):
        topic = "project_topic/automated_building_inspection_system/location"
        payload = f"{pose.position.x},{pose.position.y},{pose.position.z}"
        self.mqtt_client.publish(topic, payload)
        rospy.loginfo(f"Published robot location: {payload}")

    def shutdown(self):
        rospy.loginfo("Stopping")
        self.mqtt_client.disconnect()

if __name__ == '__main__':
    try:
        GoForwardAvoid()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
