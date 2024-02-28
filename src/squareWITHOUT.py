#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''

import rospy
from geometry_msgs.msg import Twist
from math import radians

class DrawASquare():
    def __init__(self):
        # initialize
        rospy.init_node('drawasquare', anonymous=False)

        # What to do you ctrl + c
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # 5 HZ
        r = rospy.Rate(5)

        # create two different Twist() variables. One for moving forward. One for turning 45 degrees.

        # let's go forward at 0.5 m/s for 2 seconds (1 meter)
        move_cmd = Twist()
        move_cmd.linear.x = 0.3
        # by default angular.z is 0 so setting this isn't required

        # let's turn at 90 deg/s
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(90)  # 90 deg/s in radians/s

        # keep drawing squares until count reaches 4 (one complete square)
        count = 0
        while not rospy.is_shutdown() and count < 4:
            # go forward for 2 seconds (10 x 5 HZ)
            rospy.loginfo("Going Straight")
            for x in range(0, 10):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            # turn 90 degrees
            rospy.loginfo("Turning")
            for x in range(0, 10):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()

            count += 1

        # stop turtlebot after drawing one square
        rospy.loginfo("Drawing one square completed. Stop TurtleBot.")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Drawing Squares")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        DrawASquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")

