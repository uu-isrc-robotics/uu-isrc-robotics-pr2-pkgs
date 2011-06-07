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

# Author: Lorenzo Riano <lorenzo.riano@gmail.com> (based on code from Jon Scholz)

import roslib
roslib.load_manifest("pr2_control_utilities")
import rospy
import actionlib
import math
import exceptions

from geometry_msgs.msg import Twist, Pose, PoseStamped, TwistStamped, PointStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import *
from motion_planning_msgs.srv import (FilterJointTrajectoryWithConstraints,
                                      FilterJointTrajectoryWithConstraintsRequest, 
                                      FilterJointTrajectoryWithConstraintsResponse)

class RobotState(object):
    '''
    A class to keep track of all the joint positions, divided by category.
    
    Use self.left_arm_positions right_arm_positions and self.head_positions to access the joint values.
    '''
    
    left_joint_names = ('l_shoulder_pan_joint', 
                           'l_shoulder_lift_joint',
                           'l_upper_arm_roll_joint',
                           'l_elbow_flex_joint',
                           'l_forearm_roll_joint',
                           'l_wrist_flex_joint',
                           'l_wrist_roll_joint')
    right_joint_names = ('r_shoulder_pan_joint', 
                           'r_shoulder_lift_joint',
                           'r_upper_arm_roll_joint',
                           'r_elbow_flex_joint',
                           'r_forearm_roll_joint',
                           'r_wrist_flex_joint',
                           'r_wrist_roll_joint')
    
    l_gripper_names = ('l_gripper_joint',)        
    r_gripper_names = ('r_gripper_joint',)
    
    head_joint_names = ('head_pan_joint', 'head_tilt_joint')
    
    torso_joint_names = ("torso_lift_joint",)
        
    all = (left_joint_names + 
           right_joint_names + 
           head_joint_names + 
           l_gripper_names +  
           r_gripper_names +
           torso_joint_names
           )
    
    def __init__(self):
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback)
        
        self.left_arm_positions = []
        self.right_arm_positions = []
        self.head_positions = []
        self.l_gripper_positions = []
        self.r_gripper_positions = []
        self.torso_position = []
        
        self.r_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)        
        self.l_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.r_gripper_client = actionlib.SimpleActionClient("r_gripper_controller/gripper_action", Pr2GripperCommandAction)        
        self.l_gripper_client = actionlib.SimpleActionClient("l_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.torso_client = actionlib.SimpleActionClient("torso_controller/position_joint_action", SingleJointPositionAction)
        self.head_pointer_client = actionlib.SimpleActionClient("head_traj_controller/point_head_action", PointHeadAction)
        self.head_client = rospy.Publisher('/head_traj_controller/command', JointTrajectory, latch=True)
        
        rospy.loginfo("Waiting for trajectory filter service")
        try:
            rospy.wait_for_service("/trajectory_filter/filter_trajectory_with_constraints",1)
        except rospy.ROSException:
            rospy.logwarn("No trajectory server found!")
            self.filter_service = None
        else:
            self.filter_service = rospy.ServiceProxy("/trajectory_filter/filter_trajectory_with_constraints",
                                                 FilterJointTrajectoryWithConstraints)
        
        # Some seemingly very important waits........
        rospy.loginfo("Waiting for joint trajectory actions")
        self.r_arm_client.wait_for_server()
        self.l_arm_client.wait_for_server()
        self.r_gripper_client.wait_for_server()
        self.l_gripper_client.wait_for_server()
        self.head_pointer_client.wait_for_server()
        self.torso_client.wait_for_server()
        
        self.r_arm_client.cancel_all_goals()
        self.l_arm_client.cancel_all_goals()
        self.r_gripper_client.cancel_all_goals()
        self.l_gripper_client.cancel_all_goals()
        self.head_pointer_client.cancel_all_goals()
        self.torso_client.cancel_all_goals()
        
        rospy.loginfo("Robot State is ready")
    
    def __getstate__(self):
        '''
        Used for pickle.
        '''
        return "bogus"
    
    def __setstate__(self, state):        
        '''
        Used for pickle.
        '''
        #state is bogus
        self.__init__()

    
    def joint_states_callback(self, msg):
        self.left_arm_positions = []
        for joint in self.left_joint_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
            index = msg.name.index(joint)
            self.left_arm_positions.append(msg.position[index])
            
        self.right_arm_positions = []
        for joint in self.right_joint_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
            index = msg.name.index(joint)
            self.right_arm_positions.append(msg.position[index])
            
        self.head_positions = []
        for joint in self.head_joint_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
            index = msg.name.index(joint)
            self.head_positions.append(msg.position[index])
        
        self.l_gripper_positions = []
        for joint in self.l_gripper_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
            index = msg.name.index(joint)
            self.l_gripper_positions.append(msg.position[index])    
        
        self.r_gripper_positions = []
        for joint in self.r_gripper_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
            index = msg.name.index(joint)
            self.r_gripper_positions.append(msg.position[index])
            
        self.torso_position = []
        for joint in self.torso_joint_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
            index = msg.name.index(joint)
            self.torso_position.append(msg.position[index])
        
        
class PR2JointMover(object):
    '''
    This is a convenience class to control the PR2 joints. 
    The joints can be saved and loaded from a file (see the PoseSaver class in save.py
    '''
    def __init__(self, robot_state, name=None):
        '''
        
        @param robot_state: a RobotState instance.
        @param name: an optional name.
        '''
        
        self.robot_state = robot_state
        if name is None:
            self.name = "Default"
        else:
            self.name = name
        self.time_to_reach = 5.0
        
        self.r_arm_client = robot_state.r_arm_client       
        self.l_arm_client = robot_state.l_arm_client 
        self.r_gripper_client = robot_state.r_gripper_client         
        self.l_gripper_client = robot_state.l_gripper_client 
        self.head_pointer_client = robot_state.head_pointer_client
        self.head_client = robot_state.head_client 
        self.torso_client = robot_state.torso_client
        self.filter_service = robot_state.filter_service
                
        self.__target_left_arm = []
        self.__target_right_arm = []
        self.__target_left_gripper = []
        self.__target_right_gripper = []
        self.__target_head = []
        self.__target_torso = []
        
        self.l_arm_done = False
        self.r_arm_done = False
        self.l_gripper_done = False
        self.r_gripper_done = False
        self.head_done = True
        self.torso_done = False
    
    def __getstate__(self):
        return (self.robot_state,
                self.name,
                self.__target_head,
                self.__target_left_arm,
                self.__target_left_gripper,
                self.__target_right_arm,
                self.__target_right_gripper)
    
    def __setstate__(self, state):
        (self.robot_state,
         self.name,
         self.__target_head,
         self.__target_left_arm,
         self.__target_left_gripper,
         self.__target_right_arm,
         self.__target_right_gripper) = state
        
        #bits that couldn't be pickled
        self.r_arm_client = self.robot_state.r_arm_client       
        self.l_arm_client = self.robot_state.l_arm_client 
        self.r_gripper_client = self.robot_state.r_gripper_client         
        self.l_gripper_client = self.robot_state.l_gripper_client 
        self.head_pointer_client = self.robot_state.head_pointer_client
        self.head_client = self.robot_state.head_client
        self.filter_service = self.robot_state.filter_service
        
        self.l_arm_done = False
        self.r_arm_done = False
        self.l_gripper_done = False
        self.r_gripper_done = False
        self.head_done = True
        
        
    def done(self):
        
        return self.l_arm_done and \
               self.r_arm_done and \
               self.l_gripper_done and \
               self.r_gripper_done and \
               self.head_done and \
               self.torso_done
    
    def __l_arm_done_cb(self, state, result):
        self.l_arm_done = True    
    def __r_arm_done_cb(self, state, result):
        self.r_arm_done = True
    def __l_gripper_done_cb(self, state, result):
        self.l_gripper_done = True
    def __r_gripper_done_cb(self, state, result):
        self.r_gripper_done = True
    def __head_done_cb(self):
        self.head_done = True
    def __torso_done_cb(self):
        self.torso_done = True
    
    def execute(self):
        '''
        Move to all the joint positions as previously read using parse_bookmark_file
        '''
        rospy.loginfo("Executing mover %s"%self.name)
#        self.l_arm_done = False
#        self.r_arm_done = False
#        self.l_gripper_done = False
#        self.r_gripper_done = False
#        self.torso_done = False
#        self.head_done = True
                
        self.set_arm_state(self.__target_left_arm, 'l_arm')
        self.set_arm_state(self.__target_right_arm, 'r_arm')
        self.set_head_state(self.__target_head)
        self.set_gripper_state(self.__target_left_gripper, 'l_gripper')
        self.set_gripper_state(self.__target_right_gripper, 'r_gripper')
        self.set_torso_state(self.__target_torso)

    def execute_and_wait(self):
        '''
        A blocking version of execute.
        '''
        self.execute()
        while not self.done() and not rospy.is_shutdown():
            rospy.sleep(0.01)   

    def __create_spin_command(self, arm):
        if arm == 'l':
            jnts = self.robot_state.left_arm_positions 
        if arm == 'r':
            jnts = self.robot_state.right_arm_positions 
        
        command = JointTrajectory()
        command.joint_names = ['%s_shoulder_pan_joint' % arm[0], 
                               '%s_shoulder_lift_joint' % arm[0],
                               '%s_upper_arm_roll_joint' % arm[0],
                               '%s_elbow_flex_joint' % arm[0],
                               '%s_forearm_roll_joint' % arm[0],
                               '%s_wrist_flex_joint' % arm[0],
                               '%s_wrist_roll_joint' % arm[0]]
        
        jnts[-1] += math.pi    
        command.points.append(JointTrajectoryPoint(
            positions=jnts,
            velocities = [0.0] * (len(command.joint_names)),
            accelerations = [],
            time_from_start =  rospy.Duration(0.1)))

        goal = JointTrajectoryGoal()
        goal.trajectory = command
        return goal
        
    def spin_wrists(self, ntimes):
        '''
        Rotates the joints ntimes * pi degrees.
        @param ntimes:
        '''
        
        for _ in xrange(ntimes):
            goal_r = self.__create_spin_command("r")
            goal_l = self.__create_spin_command("l")
            
            self.l_arm_client.send_goal(goal_l)
            self.r_arm_client.send_goal(goal_r)
            
            self.r_arm_client.wait_for_result()
        

    def set_arm_state(self, jvals, arm, wait = False, ):
        """ Sets goal for indicated arm (r_arm/l_arm) using provided joint values"""
        # Build trajectory message
        
        if len(jvals) == 0:
            rospy.logwarn("No %s_arm goal given" % arm[0])
            if arm[0] == "l":
                self.l_arm_done = True
            elif arm[0] == "r":
                self.r_arm_done = True
            return
        
#        rospy.loginfo("Senging %s_joints: %s" %(arm[0], str(jvals)))
        
        command = JointTrajectory()
        command.joint_names = ['%s_shoulder_pan_joint' % arm[0], 
                               '%s_shoulder_lift_joint' % arm[0],
                               '%s_upper_arm_roll_joint' % arm[0],
                               '%s_elbow_flex_joint' % arm[0],
                               '%s_forearm_roll_joint' % arm[0],
                               '%s_wrist_flex_joint' % arm[0],
                               '%s_wrist_roll_joint' % arm[0]]
        
        if arm[0] == "l":
            client = self.l_arm_client
            self.l_arm_done = False
        elif arm[0] == "r":
            client = self.r_arm_client
            self.r_arm_done = False

        
        command.points.append(JointTrajectoryPoint(
            positions=jvals,
            velocities = [0.0] * (len(command.joint_names)),
            accelerations = [],
            time_from_start =  rospy.Duration(self.time_to_reach)))
        #command.header.stamp = rospy.Time.now()

        goal = JointTrajectoryGoal()
        goal.trajectory = command
        
#        rospy.loginfo("Sending command to %s" % arm)
        if arm[0] == "l":
            client.send_goal(goal, done_cb=self.__l_arm_done_cb)       
        elif arm[0] == "r":
            client.send_goal(goal, done_cb=self.__r_arm_done_cb)
        
        if wait:
            client.wait_for_result()
            
    def execute_trajectory(self, trajectory, arm, wait=False):
        command = JointTrajectory()
        command.joint_names = ['%s_shoulder_pan_joint' % arm[0], 
                               '%s_shoulder_lift_joint' % arm[0],
                               '%s_upper_arm_roll_joint' % arm[0],
                               '%s_elbow_flex_joint' % arm[0],
                               '%s_forearm_roll_joint' % arm[0],
                               '%s_wrist_flex_joint' % arm[0],
                               '%s_wrist_roll_joint' % arm[0]]
        
        if arm[0] == "l":
            client = self.l_arm_client
            self.l_arm_done = False
        elif arm[0] == "r":
            client = self.r_arm_client
            self.r_arm_done = False
            
        for jvals in trajectory:
            command.points.append(JointTrajectoryPoint(
                                positions=jvals,
                                velocities = [],
                                accelerations = [],
                                time_from_start =  rospy.Duration(0)))
        #command.header.stamp = rospy.Time.now()

        
        rospy.loginfo("Sending request to trajectory filter")
        req = FilterJointTrajectoryWithConstraintsRequest()
        req.trajectory = command
        req.allowed_time = rospy.Duration(1.)
        reply = self.filter_service.call(req)
        
        if reply.error_code.val != reply.error_code.SUCCESS:
            rospy.logerr("Filter trajectory returns %d"%reply.error_code.val)
            return

        goal = JointTrajectoryGoal()
        goal.trajectory = reply.trajectory
        
        if arm[0] == "l":
            client.send_goal(goal, done_cb=self.__l_arm_done_cb)       
        elif arm[0] == "r":
            client.send_goal(goal, done_cb=self.__r_arm_done_cb)
        
        if wait:
            client.wait_for_result()

    def close_right_gripper(self, wait=False):
        self.set_gripper_state([0.025307], "r",wait)    
    def open_right_gripper(self, wait=False):
        self.set_gripper_state([0.090474], "r",wait)
    def close_left_gripper(self, wait=False):
        self.set_gripper_state([0.025307], "l",wait)    
    def open_left_gripper(self, wait=False):
        self.set_gripper_state([0.090474], "l",wait)

    def set_gripper_state(self, jval, gripper, wait=False):
        """ Sets goal for indicated gripper (r_gripper/l_gripper) using provided joint angle"""
        # Build trajectory message
        if not (type(jval) is list):
            jval = [jval] 
        
        if len(jval) == 0:
            rospy.logwarn("No %s_gripper goal given" % gripper[0])
            if gripper[0] == "l":
                self.l_gripper_done = True
            else:
                self.r_gripper_done = True
            return
        
        if gripper[0] == "l":
            client = self.l_gripper_client
            self.l_gripper_done = False
        elif gripper[0] == "r":            
            client = self.r_gripper_client
            self.r_gripper_done = False        

        
        goal = Pr2GripperCommandGoal()
        goal.command.max_effort = -1
        goal.command.position = jval[0]
        
        if gripper[0] == "l":
            client.send_goal(goal, done_cb=self.__l_gripper_done_cb)
        
        elif gripper[0] == "r":
            client.send_goal(goal, done_cb=self.__r_gripper_done_cb)
        if wait:
            client.wait_for_result()

    def set_head_state(self, jvals):
        """ Sets goal for head using provided joint values"""
        # Build trajectory message
        
        self.head_done = True
        
        if len(jvals) == 0:
            rospy.logwarn("No head target given")
            return
        
        head_goal = JointTrajectory()
        head_goal.joint_names.append('head_pan_joint')
        head_goal.joint_names.append('head_tilt_joint')
        
        head_goal.points.append(JointTrajectoryPoint(
            positions=jvals,
            velocities = [0.0001] * (len(head_goal.joint_names)),
#            velocities = [],
            accelerations = [],
            time_from_start =  rospy.Duration(1.0))
        )

        try:
            self.head_client.publish(head_goal)
        except:
            rospy.logerr("failed to publish head position!")

    def point_head_gripper(self, gripper):
        goal = PointHeadGoal()
        if gripper == "l_gripper":
            goal.target.header.frame_id = "l_wrist_roll_link"
        else:
            goal.target.header.frame_id = "r_wrist_roll_link"
        
        goal.pointing_frame = "head_plate_frame"
        goal.target.point.x = 0
        goal.target.point.y = 0
        goal.target.point.z = 0
        
        self.head_pointer_client.send_goal_and_wait(goal, rospy.Duration(2.0))

    def set_torso_state(self, jval, wait=False):
        """ Sets goal for torso using provided value"""
        
        if not (type(jval) is list):
            jval = [jval] 
        
        if len(jval) == 0:
            rospy.logwarn("No torso target given!")
            self.torso_done = True
            return
        
        #HACK: it looks like with a value outside this range the actionserver doesn't return..
        if jval[0] < 0.012:
            jval[0] = 0.2
        if jval[0] > 0.3:
            jval[0] = 0.3
        self.torso_done = False
        
        goal = SingleJointPositionGoal()
        goal.position = jval[0]
        goal.min_duration = rospy.Duration(self.time_to_reach)
        
        self.torso_client.send_goal(goal, done_cb=self.__torso_done_cb())
        if wait:
            self.torso_client.wait_for_result()

    def parse_bookmark_file(self, bfile):
        '''
        The file where joint positions are stored. See save.py.
        
        Example format is:
        r_arm:-0.105340072857, -0.188306789361, -0.271993550558, -1.24563923099, -1.35868478783, -1.30406916379, 1.35572398144
        l_arm:0.0680096524826, -0.187597545632, 0.00351125465628, -1.27487277927, 1.53126845039, -1.64598203412, 3.2352524133
        r_gripper: 0.037678
        l_gripper: 0.097985
        head:-0.0826166032369, 0.12723450247
         
        @param bfile: the string name of the file to open
        '''
        
        if type(bfile) is str:
            f = open(bfile,'r')
        else:
            f = bfile
        
        try:
            while True:        
                jvals = []
                l = f.readline()
                l.strip("\n")
                if len(l) == 0:
                    #This is the end of the file, let's signal that we found it
                    return False
                if l.find("arm") != -1:
                    jvals_str = l.split(":")[1]
                    jvals = map(lambda x: float(x),jvals_str.strip("\n").split(","))
                    arm = l.split(":")[0]
                    if arm[0] == 'l':
                        self.target_left_arm = jvals
                    elif arm[0] == 'r':
                        self.target_right_arm = jvals
                    else:
                        rospy.logerr("Error, unkown joint: %s"%arm)
                if l.find("gripper") != -1:
                    jval_str = l.split(":")[1]
                    jval = map(lambda x: float(x),jval_str.strip("\n").split(","))[0]
                    gripper = l.split(":")[0]
                    if gripper[0] == 'l':
                        self.target_left_gripper = [jval]
                    elif gripper[0] == 'r':
                        self.target_right_gripper = [jval]
                    else:
                        rospy.logerr("Error, unkown joint: %s"%gripper)                
                if l.find("head") != -1:
                    jvals_str = l.split(":")[1]
                    jvals = map(lambda x: float(x),jvals_str.strip("\n").split(","))
                    self.target_head = jvals
                if l.find("torso") != -1:
                    self.target_torso = [float(l.split(":")[1])]
                if l.find("time") != -1:
                    self.time_to_reach = float(l.split(":")[1])                    
                if l.find("label") != -1:
                    self.name = l.split(":")[1]
                    break
        except exceptions.Exception, e:
            rospy.logerr(e.message)
            return False
#
        if type(bfile) is str:
            f.close()

        return True

    def clear_targets(self):
        self.__target_head = []
        self.__target_left_arm = []
        self.__target_right_arm = []
        self.__target_right_gripper = []
        self.__target_left_gripper = []
        self.__target_torso = []
        
    def store_targets(self, jstate=False):
        '''
        Store the current joints as a target
        '''
        if not jstate:
            self.target_head = self.robot_state.head_positions
            self.target_left_arm = self.robot_state.left_arm_positions
            self.target_right_arm = self.robot_state.right_arm_positions            
            self.target_left_gripper = self.robot_state.l_gripper_positions            
            self.target_right_gripper = self.robot_state.r_gripper_positions
            self.target_torso = self.robot_state.torso_position            
        else:
            self.__target_head = []
            for name in self.robot_state.head_joint_names:
                try:
                    i = jstate.name.index(name)
                    self.__target_head.append(jstate.position[i])
                except ValueError:
                    i = -1
                    rospy.logerr("this shouldn't have happened")
                    return

            self.__target_left_arm = []
            for name in self.robot_state.left_joint_names:
                try:
                    i = jstate.name.index(name)
                    self.__target_left_arm.append(jstate.position[i])
                except ValueError:
                    i = -1
                    rospy.logerr("this shouldn't have happened")
                    return

            self.__target_right_arm = []
            for name in self.robot_state.right_joint_names:
                try:
                    i = jstate.name.index(name)
                    self.__target_right_arm.append(jstate.position[i])
                except ValueError:
                    i = -1
                    rospy.logerr("this shouldn't have happened")
                    return
            
            self.__target_left_gripper = []
            for name in self.robot_state.l_gripper_names:
                try:
                    i = jstate.name.index(name)
                    self.__target_left_gripper.append(jstate.position[i])
                except ValueError:
                    i = -1
                    rospy.logerr("this shouldn't have happened")
                    return
            
            self.__target_right_gripper = []
            for name in self.robot_state.r_gripper_names:
                try:
                    i = jstate.name.index(name)
                    self.__target_right_gripper.append(jstate.position[i])
                except ValueError:
                    i = -1
                    rospy.logerr("this shouldn't have happened")
                    return
                
            self.__target_torso = []
            for name in self.robot_state.torso_joint_names:
                try:
                    i = jstate.name.index(name)
                    self.__target_torso.append(jstate.position[i])
                except ValueError:
                    i = -1
                    rospy.logerr("this shouldn't have happened")
                    return
        
    def write_targets(self, bfile):
        '''
        Write on the file the previously stored target joints
        @param bfile: either a string or an open file object
        '''
        if type(bfile) is str:
            f = open(bfile,'w')
        else:
            f = bfile
        
        if len(self.__target_left_arm) > 0:
            f.write("l_arm:")
            f.write(", ".join(str(i) for i in self.__target_left_arm))
            f.write("\n")
        
        if len(self.__target_right_arm) > 0:
            f.write("r_arm:")
            f.write(", ".join(str(i) for i in self.__target_right_arm))
            f.write("\n")
        
        if len(self.__target_left_gripper) > 0:
            f.write("l_gripper:")
            f.write(", ".join(str(i) for i in self.__target_left_gripper))
            f.write("\n")

        if len(self.__target_right_gripper) > 0:        
            f.write("r_gripper:")
            f.write(", ".join(str(i) for i in self.__target_right_gripper))
            f.write("\n")

        if len(self.__target_head) > 0:        
            f.write("head:")
            f.write(", ".join(str(i) for i in self.__target_head))
            f.write("\n")

        if len(self.__target_torso) > 0:        
            f.write("torso:")
            f.write(", ".join(str(i) for i in self.__target_torso))
            f.write("\n")
        
        f.write("time: %f\n"% self.time_to_reach)
        f.write("label: %s\n" % self.name)

    def __str__(self):
        out = ""
        
        out += ("l_arm:")
        out += (", ".join(str(i) for i in self.__target_left_arm))
        out += ("\n")
        
        out += ("r_arm:")
        out += (", ".join(str(i) for i in self.__target_right_arm))
        out += ("\n")
        
        out += ("l_gripper:")
        out += (", ".join(str(i) for i in self.__target_left_gripper))
        out += ("\n")
        
        out += ("r_gripper:")
        out += (", ".join(str(i) for i in self.__target_right_gripper))
        out += ("\n")
        
        out += ("head:")
        out += (", ".join(str(i) for i in self.__target_head))
        out += ("\n")
        
        out += ("torso:")
        out += (", ".join(str(i) for i in self.__target_torso))
        out += ("\n")
        
        out += ("time: %f\n"% self.time_to_reach)
        out += ("label: %s\n" % self.name)
        
        return out

    def get_target_left_arm(self):
        return self.__target_left_arm


    def get_target_right_arm(self):
        return self.__target_right_arm


    def get_target_left_gripper(self):
        return self.__target_left_gripper


    def get_target_right_gripper(self):
        return self.__target_right_gripper


    def get_target_head(self):
        return self.__target_head


    def get_target_torso(self):
        return self.__target_torso


    def set_target_left_arm(self, value):
        self.__target_left_arm = value
        if type(value) is list:
            self.__target_left_arm = value
        else:
            self.__target_left_arm = [value]
        if len(self.__target_left_arm) != 7:
            rospy.logerr("Wrong lenght of targets, expected %d, got %d" %(7,len(self.__target_left_arm)))
            self.__target_left_arm = []

    def set_target_right_arm(self, value):
        self.__target_right_arm = value
        if type(value) is list:
            self.__target_right_arm = value
        else:
            self.__target_right_arm = [value]
        if len(self.__target_right_arm) != 7:
            rospy.logerr("Wrong lenght of targets, expected %d, got %d" %(7,len(self.__target_right_arm)))
            self.__target_right_arm = []

    def set_target_left_gripper(self, value):
        if type(value) is list:
            self.__target_left_gripper = value
        else:
            self.__target_left_gripper = [value]
        if len(self.__target_left_gripper) != 1:
            rospy.logerr("Wrong lenght of targets, expected %d, got %d" %(1,len(self.__target_left_gripper)))
            self.__target_left_gripper = []


    def set_target_right_gripper(self, value):
        if type(value) is list:
            self.__target_right_gripper = value
        else:
            self.__target_right_gripper = [value]
        if len(self.__target_right_gripper) != 1:
            rospy.logerr("Wrong lenght of targets, expected %d, got %d" %(1,len(self.__target_right_gripper)))
            self.__target_right_gripper = []


    def set_target_head(self, value):
        if type(value) is list:
            self.__target_head = value
        else:
            self.__target_head = [value]
        if len(self.__target_head) != 2:
            rospy.logerr("Wrong lenght of targets, expected %d, got %d" %(2,len(self.__target_head)))
            self.__target_head = []

    def set_target_torso(self, value):
        if type(value) is list:
            self.__target_torso = value
        else:
            self.__target_torso = [value]
        if len(self.__target_torso) != 1:
            rospy.logerr("Wrong lenght of targets, expected %d, got %d" %(1,len(self.__target_torso)))
            self.__target_torso = []

    target_left_arm = property(get_target_left_arm, set_target_left_arm)
    target_right_arm = property(get_target_right_arm, set_target_right_arm)
    target_left_gripper = property(get_target_left_gripper, set_target_left_gripper)
    target_right_gripper = property(get_target_right_gripper, set_target_right_gripper)
    target_head = property(get_target_head, set_target_head)
    target_torso = property(get_target_torso, set_target_torso)
        

class PosesSet(object):
    '''
    This class is used to execute a set of PR2JointMover in sequence. The variable
    robot_state is a list of movers that will be executed in order.
    '''
    def __init__(self, state):
        self.movers = []
        self.robot_state = state
        
    def parse_bookmark_file(self, bfile):
        '''
        Parse a file with a sequence of joints previously saved by a @PR2JointMover. 
        @param bfile: either a 
        '''
        if type(bfile) is str:
            f = open(bfile,'r')
        else:
            f = bfile
            
        num_read = 0
        while True:
            newmover = PR2JointMover(self.robot_state)
            go_on = newmover.parse_bookmark_file(f)
            if not go_on:
                rospy.loginfo("%d entries read"%num_read)
                break
            newmover.name = newmover.name.strip("\n")
            num_read += 1           
            self.movers.append(newmover)         
    
    def exec_all(self):
        rospy.loginfo("Executing %d movers"%len(self.movers))
        for mover in self.movers:
            mover.execute_and_wait()
        

def test_move_torso():
    state = RobotState()
    mover = PR2JointMover(state)
    
    mover.set_torso_state(1.0,wait=True)
    
    mover.store_targets()
#    mover.write_targets("/home/pezzotto/tmp.stack")
    
def test_open_file():
    state = RobotState()
    mover = PR2JointMover(state)
    
    mover.clear_targets()
    mover.parse_bookmark_file("/home/pezzotto/Poses/grasp_pos_big_table.stack")
    rospy.loginfo("Mover: \n%s" % str(mover))
    
    mover.execute_and_wait()

def test_save_file():
    state = RobotState()
    mover = PR2JointMover(state)
    
    mover.store_targets()
            
    
def prepare_coffee():
    state = RobotState()
    mover = PosesSet(state)
    
    mover.parse_bookmark_file("/home/pezzotto/PrepareCofee/get_the_powder.stack")
    mover.parse_bookmark_file("/home/pezzotto/PrepareCofee/pour_kettle.stack")
    mover.parse_bookmark_file("/home/pezzotto/PrepareCofee/stir.stack")
    mover.exec_all()
    
    dance1_mover = PR2JointMover(state)
    DIR = roslib.packages.get_pkg_dir("rubiks_graph", required=True) + "/RCube/"    
    filename = os.path.join(DIR, "dance1"+".pos")
    dance1_mover.parse_bookmark_file(filename)
    dance1_mover.time_to_reach = 3
    
    dance2_mover = PR2JointMover(state)
    filename = os.path.join(DIR, "dance2"+".pos")
    dance2_mover.parse_bookmark_file(filename)
    dance2_mover.time_to_reach = 3
    
    
    dance1_mover.execute_and_wait()
    dance2_mover.execute_and_wait()
    dance2_mover.spin_wrists(10)


if __name__ == "__main__":
    import os
    
    rospy.init_node('trytest', anonymous=True)
    test_move_torso()
#    test_open_file()
    
    rospy.loginfo("Done")
    while not rospy.is_shutdown():
        rospy.sleep(0.1)    
    