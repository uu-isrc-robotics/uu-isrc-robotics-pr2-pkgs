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
from geometry_msgs.msg import PoseStamped, PointStamped

import tf
import math
import pr2_control_utilities
from sensor_msgs.msg import LaserScan

def sign(x):
    if x>=0:
        return 1.
    else:
        return -1.

def sign_zero(x):
    if x > 0:
        return 1.
    elif x < 0:
        return -1.
    else:
        return 0

class PR2BaseMover(object):
    def __init__(self, listener = None):
        if listener is None:        
            self.listener = tf.TransformListener()
        else:
            self.listener = listener
        
        self.cmd_vel_pub = rospy.Publisher("/base_controller/command", Twist)
        
        self.max_vel = 0.25
        self.min_vel = 0.0
        self.min_rot = 0.0
        self.max_rot = math.pi/4
        
        self.__base_laser = LaserScan()
        self.__tilt_laser = LaserScan()
        rospy.Subscriber("base_scan", LaserScan, self.__base_laser_cbk)
        rospy.Subscriber("tilt_scan", LaserScan, self.__tilt_laser_cbk)
        
    def __base_laser_cbk(self, laser_data):
        self.__base_laser = laser_data
    
    def __tilt_laser_cbk(self, laser_data):
        self.__tilt_laser = laser_data   
    
    def drive_to_displacement(self, pos,
                              inhibit_x = False,
                              inhibit_y = False,
                              inhibit_theta = False,
                              safety_dist = 0.3):
        '''
        Drive to the relative pos (diplacement)
        @param pos: tuple: x,y,theta        
        '''
        
        desired_pos = self.convert_point((pos[0],pos[1],0),
                                            "/base_link", 
                                            "odom_combined")
        _ , rot = self.current_position()        
        desired_rot = rot[2] + pos[2]
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.get_closest_laser_reading() <= safety_dist:
                break
            (curr_trans, curr_rot) = self.current_position()
            if not inhibit_x:
                dx = desired_pos[0] - curr_trans[0]
            else:
                dx = 0
            if not inhibit_y:
                dy = desired_pos[1] - curr_trans[1]
            else:
                dy = 0
            if not inhibit_theta:
                dtheta = desired_rot - curr_rot[2]
            else:
                dtheta = 0

            #the velocity vector has to be rotated in the base_frame reference            
            vx = dx * math.cos(-curr_rot[2]) - dy * math.sin(-curr_rot[2])
            vy = dx * math.sin(-curr_rot[2]) + dy * math.cos(-curr_rot[2])
            
            if (math.fabs(vx) < 0.05 and
                math.fabs(vy) < 0.05 and
                math.fabs(dtheta) < 0.05):
                break
            
            base_cmd = Twist()
            
            if math.fabs(vx) < self.min_vel:
                base_cmd.linear.x = sign(vx) * self.min_vel
            elif math.fabs(vx) > self.max_vel:
                base_cmd.linear.x = sign(vx) * self.max_vel
            else:
                base_cmd.linear.x = vx
        
            if math.fabs(vy) < self.min_vel:
                base_cmd.linear.y = sign(vy) * self.min_vel
            elif math.fabs(vy) > self.max_vel:
                base_cmd.linear.y = sign(vy) * self.max_vel
            else:
                base_cmd.linear.y = vy
        
            if math.fabs(dtheta) < self.min_rot:
                base_cmd.angular.z = sign(dtheta) * self.min_rot
            elif math.fabs(dtheta) > self.max_rot:
                base_cmd.angular.z = sign(dtheta) * self.max_rot
            else:
                base_cmd.angular.z = dtheta
            
            self.cmd_vel_pub.publish(base_cmd)
            rate.sleep()

    def get_closest_laser_reading(self):
        '''
        Returns the closest reading between the tilt laser and the base
        laser,
        '''
        try:
            min_base_range = min( r for r in self.__base_laser.ranges 
                                  if r > self.__base_laser.range_min)
            min_tilt_range = min( r for r in self.__tilt_laser.ranges 
                                  if r > self.__tilt_laser.range_min)
            
            return min (min_base_range, min_tilt_range)
        except ValueError:
            return 10.0
    
    def move_until_obstacle_close(self, vx=0, vy=0, vtheta=0, dist=0.2):
        '''
        Move the base at the specified velocities until the closest laser is 
        equal or less than dist  
        @param vx: forward velocity
        @param vy: lateral velocity
        @param vtheta: angular velocity
        @param dist: maximum distance from an obstacle before stopping
        '''
        curr_min = self.get_closest_laser_reading()
        rate = rospy.Rate(10)
        rospy.loginfo("Min is: %f", curr_min)
        while curr_min > dist and not rospy.is_shutdown():
            base_cmd = Twist()
            base_cmd.linear.x = vx
            base_cmd.linear.y = vy
            base_cmd.angular.z = vtheta
            self.cmd_vel_pub.publish(base_cmd)
            rate.sleep()
            curr_min = self.get_closest_laser_reading()
            rospy.loginfo("Min is: %f", curr_min)
            
            
                           
    def current_position(self, frame="/odom_combined"):
        
        return self.convert_position((0,0,0), 
                                       (0,0,0), 
                                       "/base_link", 
                                       frame)
        
    def convert_position(self, pos, rot, from_frame, to_frame):
        self.listener.waitForTransform(to_frame, from_frame,
                                        rospy.Time(0), rospy.Duration(1))

        zeropose = PoseStamped()
        zeropose.header.frame_id = from_frame
        zeropose.pose.position.x = pos[0]
        zeropose.pose.position.y = pos[1]
        zeropose.pose.position.z = pos[2]
        quaternion = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2], axes="rxyz")
        zeropose.pose.orientation.x = quaternion[0]
        zeropose.pose.orientation.y = quaternion[1]
        zeropose.pose.orientation.z = quaternion[2]
        zeropose.pose.orientation.w = quaternion[3]
        
#        zeropose.header.stamp = rospy.Time.now()
        newpose = self.listener.transformPose(to_frame, zeropose)
        trans = (newpose.pose.position.x,
                 newpose.pose.position.y,
                 newpose.pose.position.z)
        quaternion = (newpose.pose.orientation.x,
                      newpose.pose.orientation.y,
                      newpose.pose.orientation.z,
                      newpose.pose.orientation.w)
        rot = tf.transformations.euler_from_quaternion(quaternion,axes="rxyz")
        return trans,rot
        
    def convert_point(self, pos, from_frame, to_frame):
        self.listener.waitForTransform(to_frame, from_frame,
                                        rospy.Time(0), rospy.Duration(1))

        zeropose = PointStamped()
        zeropose.header.frame_id = from_frame
        zeropose.point.x = pos[0]
        zeropose.point.y = pos[1]
        zeropose.point.z = pos[2]
        
        newpose = self.listener.transformPoint(to_frame, zeropose)
        return newpose.point.x, newpose.point.y, newpose.point.z
        

def test_displacement():
    mover = PR2BaseMover()
    mover.drive_to_displacement((0.0, +0.0, -math.pi/2))
    
def test_move_until_obstacle():
    mover = PR2BaseMover()
    mover.move_until_obstacle_close(vx=0.1, dist=0.)
    return

def print_the_min():
    mover = PR2BaseMover()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("Range: %f", mover.get_closest_laser_reading())
        rate.sleep()    
    
    
    
if __name__ == "__main__":
    rospy.init_node("try_move", anonymous=True)
#    test_move_until_obstacle()
    print_the_min()
    rospy.loginfo("Done")       