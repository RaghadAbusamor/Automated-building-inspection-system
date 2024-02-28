import subprocess
import rospy
import time
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler

def launch_turtlebot():
    command = "roslaunch turtlebot_bringup minimal.launch"
    print("Launching Turtlebot with command:", command)

    try:
        turtlebot_process = subprocess.Popen(command, shell=True)
        time.sleep(5)  # Allow time for TurtleBot to initialize (adjust as needed)
        print("Turtlebot launched successfully")
        return turtlebot_process
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Error running command: {e}")
        exit(1)

def launch_amcl_demo():
    command = "roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/raghad/gmapping_01.yaml"
    print("Launching AMCL demo with command:", command)

    try:
        amcl_process = subprocess.Popen(command, shell=True)
        print("AMCL demo launched successfully")
        return amcl_process
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Error running command: {e}")
        exit(1)

def launch_rviz():
    command = "roslaunch turtlebot_rviz_launchers view_navigation.launch --screen"
    print("Launching RViz with command:", command)

    try:
        rviz_process = subprocess.Popen(command, shell=True)
        print("RViz launched successfully")
        return rviz_process
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Error running command: {e}")
        exit(1)

if __name__ == '__main__':
    try:
        os.environ['ROS_DISTRO'] = 'noetic'
        os.environ['TURTLEBOT_MAP_FILE'] = '/home/raghad/gmapping_01.yaml'

        # Launch Turtlebot, AMCL demo, RViz in parallel
        turtlebot_process = launch_turtlebot()
        amcl_process = launch_amcl_demo()
        rviz_process = launch_rviz()
        
        # Wait for processes to complete
        turtlebot_process.wait()
        amcl_process.wait()
        rviz_process.wait()

        print("Turtlebot, AMCL demo, and RViz launched successfully")

        # Send autonomous navigation goals in a square pattern
        rospy.init_node("square_navigation")
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()

        # Define the corner points of the square
        square_corners = [
            ((2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ((2.0, 2.0, 0.0), (0.0, 0.0, 0.707, 0.707)),
            ((0.0, 2.0, 0.0), (0.0, 0.0, 1.0, 0.0)),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]

        for position, orientation in square_corners:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            # Set the goal position
            goal.target_pose.pose.position.x = position[0]
            goal.target_pose.pose.position.y = position[1]
            goal.target_pose.pose.position.z = position[2]

            # Set the goal orientation
            goal.target_pose.pose.orientation.x = orientation[0]
            goal.target_pose.pose.orientation.y = orientation[1]
            goal.target_pose.pose.orientation.z = orientation[2]
            goal.target_pose.pose.orientation.w = orientation[3]

            # Send the goal and wait for completion
            move_base_client.send_goal(goal)
            move_base_client.wait_for_result()

    except rospy.ROSInterruptException:
        pass
