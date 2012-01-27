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
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetPlan 
from move_base_msgs.msg import MoveBaseAction,  MoveBaseGoal
from sensor_msgs.msg import LaserScan

import tf
import math
import utils

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
    
def fix_angle(angle):
    if angle > math.pi:
        return angle - 2. * math.pi
    elif angle < -math.pi:
        return 2.*math.pi + angle 
    else: 
        return angle

class PR2BaseMover(object):
    def __init__(self, listener = None,
                 use_controller = True,
                 use_move_base = False,
                 use_safety_dist = True,
                 base_controller_name = "/base_controller/command",
                 make_plan_service = "/move_base_local_node/make_pl",
                 move_base_action = "/move_base_local",        
                 ):

        self.use_controller = use_controller
        self.use_move_base = use_move_base
        self.use_safety_dist = use_safety_dist


        if listener is None:        
            self.listener = tf.TransformListener()
        else:
            self.listener = listener
        
        if use_controller:
            self.cmd_vel_pub = rospy.Publisher(base_controller_name, Twist)
        
        self.max_vel = 0.25
        self.min_vel = 0.0
        self.min_rot = 0.0
        self.max_rot = math.pi/4
       
        if use_move_base:
            rospy.loginfo("PR2BaseMover waiting for service %s", make_plan_service)
            rospy.wait_for_service(make_plan_service)
            self.make_plan = rospy.ServiceProxy(make_plan_service, GetPlan)

            rospy.loginfo("Waiting for the action %s", move_base_action)
            self.move_action = actionlib.SimpleActionClient(move_base_action, MoveBaseAction)
            self.move_action.wait_for_server()
        
        if use_safety_dist:
            rospy.loginfo("Waiting fo the laser services")
            self.__base_laser = LaserScan()
            self.__tilt_laser = LaserScan()
            rospy.Subscriber("base_scan", LaserScan, self.__base_laser_cbk)
            rospy.Subscriber("tilt_scan", LaserScan, self.__tilt_laser_cbk)
            self.check_dist = True
            
        rospy.loginfo("PR2BaseMover ready")


    def __base_laser_cbk(self, laser_data):
        self.__base_laser = laser_data
    
    def __tilt_laser_cbk(self, laser_data):
        self.__tilt_laser = laser_data 



    def drive_to_displacement(self, pos,
                              inhibit_x = False,
                              inhibit_y = False,
                              inhibit_theta = False,
                              safety_dist = 0.3,
                              ):
        '''
        Drive to the relative pos (diplacement)
        @param pos: tuple: x,y,theta        
        '''
        
        desired_pos = utils.convert_point(self.listener,
                                          (pos[0],pos[1],0),
                                          "/base_link", 
                                          "odom_combined")
        _ , rot = self.current_position()        
        desired_rot = fix_angle(rot[2] + pos[2])
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
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
                dtheta = fix_angle(desired_rot - curr_rot[2])
            else:
                dtheta = 0

            if self.get_closest_laser_reading() <= safety_dist:
                rospy.logwarn(""""Obstacle detected before movement could be
                              completed. The distance left was %s""", (dx,dy,dtheta))
                return False


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
        return True

    def get_closest_laser_reading(self):
        '''
        Returns the closest reading between the tilt laser and the base
        laser.

        It returns a big number if self_safety_dist is false or no lasers
        have been received yet
        '''
        if not self.use_safety_dist:
            return 1e10

        try:
            min_base_range = min( r for r in self.__base_laser.ranges
                                 if r > self.__base_laser.range_min)
            min_tilt_range = min( r for r in self.__tilt_laser.ranges
                                 if r > self.__tilt_laser.range_min)

            return min (min_base_range, min_tilt_range)
        except ValueError:
            return 1e10        

    def current_position(self, frame="/odom_combined"):
        return utils.convert_position(self.listener, 
                                      (0,0,0), 
                                      (0,0,0), 
                                      "/base_link", 
                                      frame)
    
    def current_pose_stamped(self, frame="/odom_combined"):
        return utils.convert_to_posestamped(self.listener,
                                           (0,0,0), 
                                           (0,0,0), 
                                           "/base_link", 
                                           frame)
        
    def test_reachable(self, newpos, frame="/odom_combined"):
        """
        Check if it is possible to move the robot from its current position to newpos.

        @param newpos: A PoseStamped
        @param frame: Transform newpos into this frame. It has to be allowed by move_base

        """
        if not self.use_move_base:
            rospy.logerr("Trying to call a move_base service without \
            initializing it")
            return False

        start = self.current_pose_stamped(frame)
        self.listener.waitForTransform(frame, newpos.header.frame_id,
                                       rospy.Time(0), rospy.Duration((1.0)))
        goal = self.listener.transformPose(frame, newpos)
        tolerance = 0.0
        
        res = self.make_plan(start = start, goal = goal, tolerance = tolerance)
        return len(res.plan.poses) > 0
        
    def move_to_pose(self, pos, frame="/odom_combined"):
        """
        Move the robot to pos, using the move_base actions.

        @param pos: A PoseStamped
        @param frame: Check that the frame is allowed by move_base
        """
        if not self.use_move_base:
            rospy.logerr("Trying to call a move_base service without \
            initializing it")
        self.listener.waitForTransform(frame, pos.header.frame_id,
                                       rospy.Time(0), rospy.Duration((1.0)))
        target = self.listener.transformPose(frame, pos)
        goal= MoveBaseGoal()
        goal.target_pose = target
        self.move_action.send_goal(goal)
        return self.move_action.wait_for_result()
