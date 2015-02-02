#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

class TurtlebotWanerer(object):
    
    def __init__(self):
        self.isWandering = True
        self.pub=rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.wanderingSub=rospy.Subscriber("start_wandering", String, self._startWander)
        self.bumperSub=rospy.Subscriber("turtlebot_stop", String, self._stopWander)

    def _startWander(self, msg):
        self.isWandering = True
        rate = rospy.Rate(20)
	while self.isWandering:
	    twist = Twist()
   	    twist.linear.x = 0.1
            rospy.loginfo(twist)
            self.pub.publish(twist)
            rate.sleep()

    def _stopWander(self, msg):
        self.isWandering = False

if __name__ == '__main__':
    try:
	rospy.init_node('turtlebot_wanerer', anonymous=True)
        turtlebotWanerer = TurtlebotWanerer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
