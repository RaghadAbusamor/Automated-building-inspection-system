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

class MoveInCircle():
    def __init__(self):
        rospy.init_node('move_in_circle', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
        # Set linear velocity for forward motion (adjust as needed)
        move_cmd = Twist()
        move_cmd.linear.x = 0.1  # Adjust this value to control linear velocity

        # Set angular velocity for turning (adjust as needed)
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(30)  # Adjust this value to control angular velocity for turning

        # Duration of each movement (adjust as needed)
        movement_duration = 20  # 20 iterations at 5 Hz, adjust based on your desired velocity

        r = rospy.Rate(7)

        while not rospy.is_shutdown():
            # Move forward for the specified duration
            rospy.loginfo("Moving forward in a circle")
            for _ in range(movement_duration):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            # Turn for the specified duration to create a circular path
            rospy.loginfo("Turning to continue the circle")
            for _ in range(movement_duration):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop Moving in Circle")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveInCircle()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")


