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
from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal
from actionlib import SimpleActionClient
from arm_navigation_msgs.msg import PositionConstraint, OrientationConstraint
from arm_navigation_msgs.msg import OrderedCollisionOperations, CollisionOperation
from arm_navigation_msgs.msg import AllowedContactSpecification
from arm_navigation_msgs.msg import Shape
from arm_navigation_msgs.srv import SetPlanningSceneDiff
import tf
import utils
from arm_navigation_msgs.msg import MakeStaticCollisionMapAction, MakeStaticCollisionMapGoal
from actionlib_msgs.msg import GoalStatus

import pr2_control_utilities
import math


class PR2MoveArm(object):
    def __init__(self, joint_mover):
        rospy.loginfo("Waiting for the move_arm actions")                
        self.move_right_arm_client = SimpleActionClient("move_right_arm", MoveArmAction)
        self.move_right_arm_client.wait_for_server()
        
        self.move_left_arm_client = SimpleActionClient("move_left_arm", MoveArmAction)
        self.move_left_arm_client.wait_for_server()
        
        rospy.loginfo("waiting for action make_static_collision_map action server")
        self.make_static_collision_map_client = actionlib.SimpleActionClient(
                                            'make_static_collision_map', 
                                            MakeStaticCollisionMapAction)
        self.make_static_collision_map_client.wait_for_server()
        
        rospy.loginfo("waiting for service set_planning_scene_diff")
        self.planning_scene_client = rospy.ServiceProxy(
                                "/environment_server/set_planning_scene_diff",
                                SetPlanningSceneDiff)
        self.planning_scene_client.wait_for_service()
        
        self.tf_listener = tf.TransformListener()
        self.right_ik = pr2_control_utilities.IKUtilities("right", 
                                                          tf_listener=self.tf_listener)
        self.left_ik = pr2_control_utilities.IKUtilities("left", 
                                                         tf_listener=self.tf_listener)
        self.joint_mover = joint_mover
        
        rospy.loginfo("%s is ready", self.__class__.__name__)
        
    def __move_arm(self, arm, position, orientation, frame_id,  waiting_time, 
                   ordered_collision_operations = None,
                   allowed_contacts = None):
        
        goal = MoveArmGoal()
        goal.motion_plan_request.group_name = arm
        goal.motion_plan_request.num_planning_attempts = 2
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(waiting_time/2.)
        
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame_id
        if arm == "right_arm":
            link_name = "r_wrist_roll_link"
            client = self.move_right_arm_client
        elif arm == "left_arm":
            link_name = "l_wrist_roll_link"
            client = self.move_left_arm_client
        else:
            rospy.logerr("Unknown arm: %s"%arm)
            return False
            
        position_constraint.link_name = link_name
        
        position_constraint.position.x = position[0]
        position_constraint.position.y = position[1]
        position_constraint.position.z = position[2]
        position_constraint.constraint_region_shape.type = position_constraint.constraint_region_shape.BOX
        tolerance = 2 * 0.02
        position_constraint.constraint_region_shape.dimensions = [tolerance, tolerance, tolerance]
        
        position_constraint.constraint_region_orientation.x = 0.
        position_constraint.constraint_region_orientation.y = 0.
        position_constraint.constraint_region_orientation.z = 0.
        position_constraint.constraint_region_orientation.w = 1.        
        position_constraint.weight = 1.0
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame_id
        orientation_constraint.link_name = link_name
#        orientation_constraint.type = dunno!
        orientation_constraint.orientation.x = orientation[0]
        orientation_constraint.orientation.y = orientation[1]
        orientation_constraint.orientation.z = orientation[2]
        orientation_constraint.orientation.w = orientation[3]
        
        orientation_constraint.absolute_roll_tolerance = 0.04
        orientation_constraint.absolute_pitch_tolerance = 0.04
        orientation_constraint.absolute_yaw_tolerance = 0.04
        orientation_constraint.weight = 1.0
        
        goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
        goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)
        
        if ordered_collision_operations is not None:
            rospy.loginfo("Adding ordered collisions")
            goal.operations = ordered_collision_operations
        if allowed_contacts is not None:
            rospy.loginfo("Adding allowed_contacts")
            goal.planning_scene_diff.allowed_contacts = allowed_contacts
        goal.disable_collision_monitoring = False
        
        state = client.send_goal_and_wait(goal, rospy.Duration(waiting_time))
        if state == actionlib.GoalStatus.SUCCEEDED:
            return True
        else:
            return False
    
    def update_planning_scene(self):
        self.planning_scene_client.call()
    
    def move_right_arm(self, position, orientation, frame_id,  waiting_time, 
                       ordered_collision_operations = None,
                       allowed_contacts = None):
        return self.__move_arm("right_arm", 
                               position, 
                               orientation, 
                               frame_id, 
                               waiting_time, 
                               ordered_collision_operations, 
                               allowed_contacts)
    def move_left_arm(self, position, orientation, frame_id,  waiting_time, 
                       ordered_collision_operations = None,
                       allowed_contacts = None):
        return self.__move_arm("left_arm", 
                               position, 
                               orientation, 
                               frame_id, 
                               waiting_time, 
                               ordered_collision_operations, 
                               allowed_contacts)
        
   
    def __check_ik_feasible(self, arm, pose_stamped):
        if arm == "right_arm":
            link_name = "r_wrist_roll_link"
            joint_angles = self.joint_mover.robot_state.right_arm_positions
            ik = self.right_ik 
        elif arm == "left_arm":
            link_name = "l_wrist_roll_link"
            joint_angles = self.joint_mover.robot_state.left_arm_positions
            ik = self.left_ik 
        else:
            rospy.logerr("Unknown arm: %s"%arm)
            return False

        res = ik.run_ik(pose_stamped, joint_angles, link_name, collision_aware=0)
        if res is None:
            return False
        elif len(res[0]) == 0:
            return False
        else:
            return True

    def check_ik_right_arm(self, pose_stamped):
        """
        Returns True if pose_stamped is reachable by the right arm.

        Parameters:
        pose_stamped: a PoseStamped instance
        """
        return self.__check_ik_feasible("right_arm", pose_stamped)

    def check_ik_left_arm(self, pose_stamped):
        """
        Returns True if pose_stamped is reachable by the left arm.

        Parameters:
        pose_stamped: a PoseStamped instance
        """
        return self.__check_ik_feasible("left_arm", pose_stamped)

    def __move_arm_non_collision(self, arm, position, orientation, frame_id,
                                 time_required):
        
        if arm == "right_arm":
            link_name = "r_wrist_roll_link"
            joint_angles = self.joint_mover.robot_state.right_arm_positions
            mover_arm = "right"
            ik = self.right_ik 
        elif arm == "left_arm":
            link_name = "l_wrist_roll_link"
            joint_angles = self.joint_mover.robot_state.left_arm_positions
            mover_arm = "left"
            ik = self.left_ik 
        else:
            rospy.logerr("Unknown arm: %s"%arm)
            return False
        
        end_pose = ik.lists_to_pose_stamped(position, 
                                                 orientation, 
                                                 frame_id, 
                                                 frame_id)
        
        joints, _ = ik.run_ik(end_pose, joint_angles, link_name, 
                                 collision_aware=0) 
        if joints is not None:
            old_time = self.joint_mover.time_to_reach
            self.joint_mover.time_to_reach = time_required
            self.joint_mover.set_arm_state(joints, mover_arm, wait=True)
            self.joint_mover.time_to_reach = old_time
            return True
        else:
            return False
   
    def get_right_gripper_pose(self, frame="/base_link"):
        """Returns a PoseStamped with the position/orientation of the
        right gripper.

        Parameters:
        frame: the frame to use for the returned PoseStamped.
        """
        link_name = "r_wrist_roll_link"
        return utils.convert_to_posestamped(self.tf_listener,
                                           (0,0,0), 
                                           (0,0,0), 
                                           link_name,
                                           frame, 
                                           )
    
    def get_left_gripper_pose(self, frame = "/base_link"):
        """Returns a PoseStamped with the position/orientation of the
        left gripper.

        Parameters:
        frame: the frame to use for the returned PoseStamped.
        """
        link_name = "l_wrist_roll_link"
        return utils.convert_to_posestamped(self.tf_listener,
                                           (0,0,0), 
                                           (0,0,0),
                                           link_name,
                                           frame, 
                                           )


    def move_right_arm_non_collision(self, position, orientation, 
                                     frame_id,
                                     time_required = 1.0,
                                     ):
        return self.__move_arm_non_collision("right_arm", 
                                             position, 
                                             orientation, 
                                             frame_id,
                                             time_required)        

    def move_left_arm_non_collision(self, position, orientation, 
                                     frame_id,
                                     time_required = 1.0,
                                     ):
        return self.__move_arm_non_collision("left_arm", 
                                              position, 
                                             orientation, 
                                             frame_id,
                                             time_required)
    
    def __create_trjectory_non_collision(self,
                                         arm,
                                         positions, 
                                         orientations, 
                                         frame_id,
                                         max_vel, 
                                         ignore_errors = False,
                                         normalize = True):
        if arm == "right_arm":
            link_name = "r_wrist_roll_link"
            joint_angles = self.joint_mover.robot_state.right_arm_positions
            ik = self.right_ik 
        elif arm == "left_arm":
            link_name = "l_wrist_roll_link"
            joint_angles = self.joint_mover.robot_state.left_arm_positions
            ik = self.left_ik 
        else:
            rospy.logerr("Unknown arm: %s"%arm)
            return False
        
        trajectory = []
        for i in xrange(len(positions)):
            
            pose = ik.lists_to_pose_stamped(positions[i], 
                                            orientations[i], 
                                            frame_id, 
                                            frame_id)
            
            (joints,e) = ik.run_ik(pose, joint_angles,
                                   link_name, collision_aware=0) 
            
            if (not ignore_errors) and (e != "SUCCESS"):
                rospy.logerr("IK returns error codes: %s at step %d"%(str(e),i))
                return False
            if e == "SUCCESS":
                trajectory.append(joints)
        
        if normalize:
            rospy.loginfo("Normalising trajectory")
            #trajectory = self.__normalize_trajectory(trajectory, joint_angles)
            trajectory = utils.normalize_trajectory(trajectory, joint_angles)
        else:
            rospy.loginfo("Not normalising the trajectory")
        (times, vels) = ik.trajectory_times_and_vels(trajectory, [max_vel]*7)
        return (trajectory, times, vels)
    
    def create_right_arm_trjectory_non_collision(self,
                                                 positions, 
                                                 orientations, 
                                                 frame_id,
                                                 max_vel, 
                                                 ignore_errors = False,
                                                 normalize = True
                                                 ):
        return self.__create_trjectory_non_collision("right_arm", 
                                                     positions, 
                                                     orientations, 
                                                     frame_id, 
                                                     max_vel, 
                                                     ignore_errors,
                                                     normalize)
    
    def create_left_arm_trjectory_non_collision(self,
                                                positions, 
                                                orientations, 
                                                frame_id,
                                                max_vel, 
                                                ignore_errors = False,
                                                normalize = True
                                                ):
        return self.__create_trjectory_non_collision("left_arm", 
                                                     positions, 
                                                     orientations, 
                                                     frame_id, 
                                                     max_vel, 
                                                     ignore_errors,
                                                     normalize)
    
    
    def __move_arm_trajectory_non_collision(self,
                                            arm,
                                            positions, 
                                            orientations, 
                                            frame_id,
                                            max_vel, 
                                            ignore_errors = False,
                                            normalize = True):        
        
        if arm == "right_arm":
            mover_arm = "right"
        elif arm == "left_arm":
            mover_arm = "left" 
        else:
            rospy.logerr("Unknown arm: %s"%arm)
            return False
        
        res = self.__create_trjectory_non_collision(arm, 
                                                    positions, 
                                                    orientations, 
                                                    frame_id, 
                                                    max_vel, 
                                                    ignore_errors,
                                                    normalize)
        
        if res:
            trajectory, times, vels = res 
        
        self.joint_mover.execute_trajectory(trajectory, times, vels, mover_arm, True)
        return True
    
    def move_right_arm_trajectory_non_collision(self, 
                                                positions, 
                                                orientations, 
                                                frame_id,
                                                max_vel, 
                                                ignore_errors = False,
                                                normalize = True
                                                ):
        return self.__move_arm_trajectory_non_collision("right_arm", 
                                                 positions, 
                                                 orientations, 
                                                 frame_id, 
                                                 max_vel, 
                                                 ignore_errors,
                                                 normalize)
    
    def move_left_arm_trajectory_non_collision(self, 
                                                positions, 
                                                orientations, 
                                                frame_id,
                                                max_vel, 
                                                ignore_errors = False,
                                                normalize = True,
                                                ):
        return self.__move_arm_trajectory_non_collision("left_arm", 
                                                 positions, 
                                                 orientations, 
                                                 frame_id, 
                                                 max_vel, 
                                                 ignore_errors,
                                                 normalize)
    
    
    ##normalize a trajectory (list of lists of joint angles), so that the desired angles 
    #are the nearest ones for the continuous joints (5 and 7)
    #def __normalize_trajectory(self, trajectory, current_angles):
    #    trajectory_copy = [list(angles) for angles in trajectory]
    #    for angles in trajectory_copy:
    #        angles[4] = self.__normalize_angle(angles[4], current_angles[4])
    #        angles[6] = self.__normalize_angle(angles[6], current_angles[6])
    #    return trajectory_copy

    ##normalize an angle for a continuous joint so that it's the closest version 
    #of the angle to the current angle (not +-2*pi)
    #def __normalize_angle(self, angle, current_angle):
    #    while current_angle-angle > math.pi:
    #        angle += 2*math.pi
    #    while angle - current_angle > math.pi:
    #        angle -= 2*math.pi
    #    return angle
    
    def build_collision_operations(self, object1, object2):
        msg = OrderedCollisionOperations()
        collision = CollisionOperation()
        
        collision.operation = CollisionOperation.DISABLE
        collision.object1 = object1
        collision.object2 = object2
        msg.collision_operations.append(collision)
        return msg
    
    def build_allowed_contact_specification(self, box_pose, box_dimensions):
        msg = AllowedContactSpecification()
        msg.name = "grasping_object_region"
        shape = Shape()
        shape.type = shape.BOX        
        shape.dimensions = box_dimensions
        
        msg.shape = shape
        msg.pose_stamped = box_pose
        
        msg.link_names = ["r_gripper_palm_link",
                          "r_gripper_l_finger_link", 
                          "r_gripper_r_finger_link",
                          "r_gripper_l_finger_tip_link",
                          "r_gripper_r_finger_tip_link",
                          "l_gripper_palm_link",
                          "l_gripper_l_finger_link", 
                          "l_gripper_r_finger_link",
                          "l_gripper_l_finger_tip_link",
                          "l_gripper_r_finger_tip_link"]
        return msg
    
    def take_static_map(self):
        rospy.loginfo("Taking a static collision map")
        static_map_goal = MakeStaticCollisionMapGoal()
        static_map_goal.cloud_source = "full_cloud_filtered"
        static_map_goal.number_of_clouds = 2
        self.make_static_collision_map_client.send_goal(static_map_goal)

        if not self.make_static_collision_map_client.wait_for_result(rospy.Duration(30)):
            rospy.loginfo("collision map was not formed in allowed time")
            return 0

        if self.make_static_collision_map_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("static map successfully updated")
            return 1
        else:
            rospy.loginfo("some other non-success state was reached for static collision map.  Proceed with caution.")
            return 0
    
if __name__ == "__main__":
    rospy.init_node('trytest', anonymous=True)
    move_arm = PR2MoveArm()
    
    position = (0.55 - 0.1, -0.188, 0)
    orientation = (0., 0., 0., 1.)
    
    if move_arm.move_right_arm(position, orientation, "/torso_lift_link", 120.):
        rospy.loginfo("OK")
    else:
        rospy.loginfo("bad")
            
        
        
