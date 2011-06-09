#! /usr/bin/env python
# Copyright (c) 2010, Lorenzo Riano.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#         * Redistributions of source code must retain the above copyright
#             notice, this list of conditions and the following disclaimer.
#         * Redistributions in binary form must reproduce the above copyright
#             notice, this list of conditions and the following disclaimer in the
#             documentation and/or other materials provided with the distribution.
#         * Neither the name of the Lorenzo Riano. nor the names of its
#             contributors may be used to endorse or promote products derived from
#             this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Lorenzo Riano <lorenzo.riano@gmail.com>

import roslib
roslib.load_manifest("pr2_control_utilities")

import rospy
from geometry_msgs.msg import Twist
import tf
import math
import pr2_control_utilities

def sign(x):
    if x>0:
        return 1.
    else:
        return -1.

class PR2BaseMover(object):
    def __init__(self):
        self.listener = tf.TransformListener()
        self.cmd_vel_pub = rospy.Publisher("/base_controller/command", Twist)
        
        self.max_vel = 0.25
        self.min_vel = 0.1
        self.min_rot = 0.1
        self.max_rot = 0.1
       
    def drive_to_displacement(self, pos, inhibit_x = False, inhibit_y = False, inhibit_theta = False):
        '''
        Drive to the relative pos (diplacement)
        @param pos: tuple: x,y,theta        
        '''
        
        self.listener.waitForTransform("base_footprint", "odom_combined",
                                        rospy.Time(0), rospy.Duration(1))        
        
        (start_trans,start_rot) = self.listener.lookupTransform("base_footprint", 
                                                                "odom_combined",
                                                                rospy.Time(0))
        
        desired_trans = (start_trans[0] + pos[0],
                         start_trans[1] + pos[1],
                         start_trans[2])
        desired_rot = (start_rot[0],
                       start_rot[1],
                       start_rot[2] + pos[2])
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            (curr_trans, curr_rot) = self.listener.lookupTransform("base_footprint", 
                                                                   "odom_combined",
                                                                   rospy.Time(0))
            if not inhibit_x:
                dx = curr_trans[0] - desired_trans[0]
            else:
                dx = 0
            if not inhibit_y:
                dy = curr_trans[1] - desired_trans[1]
            else:
                dy = 0
            if not inhibit_theta:
                dtheta = curr_rot[2] - desired_rot[2]
            else:
                dtheta = 0
            
#            rospy.loginfo("DX: %s"%str((dx,dy,dtheta)))
            
            if (math.fabs(dx) < 0.01 and
                math.fabs(dy) < 0.01 and
                math.fabs(dtheta) < 0.01):                
                break
            
            base_cmd = Twist()
            
            if math.fabs(dx) < 0.01:
                base_cmd.linear.x = 0
            else:
                if math.fabs(dx) < self.min_vel:
                    base_cmd.linear.x = sign(dx) * self.min_vel
                elif math.fabs(dx) > self.max_vel:
                    base_cmd.linear.x = sign(dx) * self.max_vel
                else:
                    base_cmd.linear.x = dx
            
            if math.fabs(dy) < 0.01:
                base_cmd.linear.y = 0
            else:
                if math.fabs(dy) < self.min_vel:
                    base_cmd.linear.y = sign(dy) * self.min_vel
                elif math.fabs(dy) > self.max_vel:
                    base_cmd.linear.y = sign(dy) * self.max_vel
                else:
                    base_cmd.linear.y = dy
            
            if math.fabs(dtheta) < 0.01:
                base_cmd.angular.z = 0
            else:
                if math.fabs(dtheta) < self.min_rot:
                    base_cmd.angular.z = sign(dtheta) * self.min_rot
                elif math.fabs(dx) > self.max_rot:
                    base_cmd.angular.z = sign(dtheta) * self.max_rot
                else:
                    base_cmd.angular.z = dtheta
            
#            rospy.loginfo("CMD: %s" % str(base_cmd))
            self.cmd_vel_pub.publish(base_cmd)
            rate.sleep()
                            

def test_move():
    mover = PR2BaseMover()
    mover.drive_to_displacement((0,-0.0,0.1))
    
if __name__ == "__main__":
    rospy.init_node("try_move", anonymous=True)
    test_move()
    rospy.loginfo("Done")       