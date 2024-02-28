#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist

class GoForwardAndTurn():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)

        # Initialize action client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

        # Define and send the first goal to move forward by 3 meters
        first_goal = MoveBaseGoal()
        first_goal.target_pose.header.frame_id = 'base_link'
        first_goal.target_pose.header.stamp = rospy.Time.now()
        first_goal.target_pose.pose.position.x = 2.9
        first_goal.target_pose.pose.orientation.w = 1

        # Set linear and angular velocities for the first goal
        twist = Twist()
        twist.linear.x = 0.1  # Decrease this value to reduce linear speed
        twist.angular.z = 0.05  # Decrease this value to reduce angular speed
        first_goal.target_pose.twist = twist

        self.move_base.send_goal(first_goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move forward 2 meters for some reason")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved 2 meters forward")

        # Define and send the second goal to move left by 1 meter
        second_goal = MoveBaseGoal()
        second_goal.target_pose.header.frame_id = 'base_link'
        second_goal.target_pose.header.stamp = rospy.Time.now()
        second_goal.target_pose.pose.position.y = 2.2
        second_goal.target_pose.pose.orientation.w = 0.5

        # Set linear and angular velocities for the second goal
        twist = Twist()
        twist.linear.x = 0.1  # Adjust linear speed if needed
        twist.angular.z = 0.05  # Adjust angular speed if needed
        second_goal.target_pose.twist = twist

        self.move_base.send_goal(second_goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move left 2 meters for some reason")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved left 2 meters")

        # Define and send the third goal to move backward by 1 meter
        third_goal = MoveBaseGoal()
        third_goal.target_pose.header.frame_id = 'base_link'
        third_goal.target_pose.header.stamp = rospy.Time.now()
        third_goal.target_pose.pose.position.x = -2
        third_goal.target_pose.pose.orientation.w = 1.0

        # Set linear and angular velocities for the third goal
        # Adjust the twist values as needed
        third_goal.target_pose.twist = Twist()

        self.move_base.send_goal(third_goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move backward 2 meters for some reason")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved backward 2 meters")

        # Define and send the fourth goal to move left by 1 meter
        fourth_goal = MoveBaseGoal()
        fourth_goal.target_pose.header.frame_id = 'base_link'
        fourth_goal.target_pose.header.stamp = rospy.Time.now()
        fourth_goal.target_pose.pose.position.y = -1
        fourth_goal.target_pose.pose.orientation.w = 0.5

        # Set linear and angular velocities for the fourth goal
        # Adjust the twist values as needed
        fourth_goal.target_pose.twist = Twist()

        self.move_base.send_goal(fourth_goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move left 1 meter for some reason")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved left 1 meter")

    def shutdown(self):
        rospy.loginfo("Stop")

if __name__ == '__main__':
    try:
        GoForwardAndTurn()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
