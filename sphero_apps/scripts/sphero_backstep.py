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
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SpheroBackStapper(object):
    
    def __init__(self):
        self.pub=rospy.Publisher('~/cmd_vel', Twist, queue_size=10)
        self.sub=rospy.Subscriber("sphero_backstep_cmd", String, self._pubCmdVel)

    def _pubCmdVel(self, msg):
        rate = rospy.Rate(10)
        timeout = time.time() + 3
        
        while True:
	    twist = Twist()
    	    twist.linear.x = 100
    	    twist.angular.z = 100
            rospy.loginfo(twist)
            self.pub.publish(twist)
            rate.sleep()
            if time.time() > timeout:
                break;
        rospy.loginfo('end of while')

if __name__ == '__main__':
    try:
	rospy.init_node('sphero_backstep', anonymous=True)
        spheroBackStapper = SpheroBackStapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
