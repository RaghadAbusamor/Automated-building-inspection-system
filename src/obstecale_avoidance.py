#!/usr/bin/env python

import subprocess
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from math import radians
from geometry_msgs.msg import Twist

class SquareAvoidObstacles():
    def __init__(self):
        # Launch TurtleBot, AMCL demo, and RViz in parallel
        turtlebot_process = subprocess.Popen("roslaunch turtlebot_bringup minimal.launch", shell=True)
        rospy.sleep(5)  # Allow time for TurtleBot to initialize (adjust as needed)

        amcl_process = subprocess.Popen("roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/raghad/gmapping_01.yaml", shell=True)
        rviz_process = subprocess.Popen("roslaunch turtlebot_rviz_launchers view_navigation.launch --screen", shell=True)
        rospy.sleep(15)  # Allow time for AMCL and RViz to launch

        # Initialize ROS node
        rospy.init_node('square_navigation', anonymous=False)

        # What to do if shut down (e.g., ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        # Wait for the move_base action server to come up
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the move_base action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

        # Set up a publisher for velocity commands
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Define the square corners
        square_corners = [
            (0.5, 0.0, 0.0),   # Go 0.5 meters forward (x-axis)
            (0.0, 0.5, radians(-90.0)),  # Go 0.5 meters left and turn -90 degrees (y-axis)
            (0.5, 0, radians(180.0)),  # Go 0.5 meters backward and turn 180 degrees (x-axis)
            (0.0, 0.5, radians(90.0)),   # Go 0.5 meters right and turn 90 degrees (y-axis)
        ]

        # Start moving in a square with obstacle avoidance
        for position in square_corners:
            self.move_to_goal(position[0], position[1], position[2])

    def move_to_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'  # Use the map frame
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0  # Keep the robot at ground level

        # Set the goal orientation
        q = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete the task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the goal for some reason")
        else:
            # Goal reached
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base reached the goal")

    def shutdown(self):
        rospy.loginfo("Stopping the TurtleBot")
        # Stop the robot when shutting down
        self.cmd_vel.publish(Twist())
        self.move_base.cancel_all_goals()

if __name__ == '__main__':
    try:
        SquareAvoidObstacles()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
