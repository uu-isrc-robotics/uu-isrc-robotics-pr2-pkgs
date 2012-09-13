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
from geometry_msgs.msg import PoseStamped, PoseArray
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker
import tf
import utils
from arm_navigation_msgs.msg import MakeStaticCollisionMapAction, MakeStaticCollisionMapGoal
from actionlib_msgs.msg import GoalStatus
from pr2_control_utilities.pr2_joint_mover import PR2JointMover
from std_srvs.srv import Empty

from dynamic_reconfigure.server import Server
from pr2_control_utilities.cfg import pr2_planningConfig

import pr2_control_utilities
import math
import random
import numpy as np


from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionFK, GetConstraintAwarePositionIK, GetConstraintAwarePositionIKRequest

class PR2MoveArm(object):
    def __init__(self, joint_mover = None):
        if joint_mover is None:
            self.joint_mover = PR2JointMover()
        else:
            self.joint_mover = joint_mover

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
        
        rospy.loginfo("waiting for service /collider_node/reset")
        self.reset_srv = rospy.ServiceProxy(
                                "/collider_node/reset",
                                Empty)
        self.reset_srv.wait_for_service()        

        self.tf_listener = tf.TransformListener()
        self.right_ik = pr2_control_utilities.IKUtilities("right",
                                                          tf_listener=self.tf_listener)
        self.right_ik.check_services_and_get_ik_info()
        
        self.left_ik = pr2_control_utilities.IKUtilities("left",
                                                         tf_listener=self.tf_listener)
        self.left_ik.check_services_and_get_ik_info()
        
        self.planner_service_name = ""
        self.parameters_server = Server(pr2_planningConfig, self.__new_parameter)

        self.arrows_pub = rospy.Publisher("~pointing_arrows",
                        Marker)

        rospy.loginfo("%s is ready", self.__class__.__name__)
        
    def __new_parameter(self, config, level):
        self.planner_service_name = config["planner_service_name"]
        rospy.loginfo("New planner service name: %s", self.planner_service_name)
        return config

    def __move_arm(self, arm, position, orientation, frame_id,  waiting_time,
                   ordered_collision_operations = None,
                   allowed_contacts = None):
        goal = MoveArmGoal()
        goal.motion_plan_request.group_name = arm
        goal.motion_plan_request.num_planning_attempts = 2
        goal.motion_plan_request.planner_id = ""
        #goal.planner_service_name = "ompl_planning/plan_kinematic_path"
        #goal.planner_service_name = "/chomp_planner_longrange/plan_path"
        #goal.planner_service_name = "/collision_proximity_server_test/get_distance_aware_plan"
        goal.planner_service_name = self.planner_service_name
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
            #goal.planning_scene_diff.allowed_contacts = allowed_contacts
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

    def reset_collision_map(self):
        rospy.loginfo("Resetting collision map")
        self.reset_srv()

    def update_planning_scene(self):
        rospy.loginfo("Updating planning scene")
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


    def find_best_arm(self, pose):
        """Given a PoseStamped, returns the best arm with which to reach that pose.
        So far it is just a simple euristic.

        Parameters:
        pose: a PoseStamped

        Returns:
        either left_arm or right_arm
        """
        assert isinstance(pose, PoseStamped)
        self.tf_listener.waitForTransform("base_link", pose.header.frame_id,
                                          rospy.Time.now(), rospy.Duration(1))
        newpose = self.tf_listener.transformPose("base_link", pose)
        if newpose.pose.position.y > 0:
            return "left_arm"
        else:
            return "right_arm"


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
        elif res[0] is None:
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

    def __move_arm_with_ik(self, arm, position, orientation, frame_id,
                                 time_required,
                                 collision_aware = 0):

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
                                 collision_aware=collision_aware)

        if joints is not None and len(joints) != 0:
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
        return self.__move_arm_with_ik("right_arm",
                                             position,
                                             orientation,
                                             frame_id,
                                             time_required)

    def move_left_arm_non_collision(self, position, orientation,
                                     frame_id,
                                     time_required = 1.0,
                                     ):
        return self.__move_arm_with_ik("left_arm",
                                              position,
                                             orientation,
                                             frame_id,
                                             time_required)

    def move_right_arm_with_ik(self, position, orientation,
                                     frame_id,
                                     time_required = 1.0,
                                     ):
        return self.__move_arm_with_ik("right_arm",
                                             position,
                                             orientation,
                                             frame_id,
                                             time_required,
                                             collision_aware=1)

    def move_left_arm_with_ik(self, position, orientation,
                                     frame_id,
                                     time_required = 1.0,
                                     ):
        return self.__move_arm_with_ik("left_arm",
                                              position,
                                             orientation,
                                             frame_id,
                                             time_required,
                                             collision_aware=1)

    def execute_JointTrajectory(self, joint_trajectory, normalize=True, max_vel = 0.2):
        """Executes a trajectory. It does not check if the trajectory is safe, nor it performs
        any interpolation or smoothing! If velocities or accelerations in joint_trajectory are not set
        then they are automatically calculated.
        
        Parameters:
        joint_trajectory: a JointTrajectory msg
        normalize: if True the continous joints are normalized
        max_vel = maximum velocity for all the joints
        
        """
        
        isinstance(joint_trajectory, JointTrajectory)
        if joint_trajectory.joint_names[0][0] == "l":
            joint_angles = self.joint_mover.robot_state.right_arm_positions
            ik = self.right_ik
        else:
            joint_angles = self.joint_mover.robot_state.right_arm_positions
            ik = self.right_ik
        
        if len(joint_trajectory.points) == 0:
            rospy.logwarn("Empty trajectory!")
            return False
        
        #create velocities if they don't exist
        if len(joint_trajectory.points[0].velocities) == 0:
            trajectory = [p.positions for p in joint_trajectory.points]
            if normalize:
                #trajectory = self.__normalize_trajectory(trajectory, joint_angles)
                trajectory = utils.normalize_trajectory(trajectory, joint_angles)            
            (times, vels) = ik.trajectory_times_and_vels(trajectory, [max_vel]*7)
            
            for i in xrange(len(joint_trajectory.points)):
                joint_trajectory.points[i].positions = trajectory[i]
                joint_trajectory.points[i].velocities = vels[i]
                joint_trajectory.points[i].time_from_start = rospy.Duration(times[i])
        
        self.joint_mover.execute_JointTrajectory(joint_trajectory)



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
                                                 ignore_errors = False,
                                                 normalize = True)

    def move_arm_trajectory(self, poses, which_arm, max_vel, 
                            ignore_errors = False, 
                            normalize = True):
        """Move an arm along a trajectory.
        
        Parameters:
        poses: a geometry_msgs/PoseStamped msg with the cartesian poses to follow
        which_arm: either left_arm or right_arm, a string
        max_vel: the maximum velocity for all the joints
        ignore_errors: if True, points along the trajectory with no IK solution will simply be discarded without raising an error
        normalize: normalize angles so that continuous joints don't spin indefinitely
        """
        assert isinstance(poses, PoseArray)
        frame_id = poses.header.frame_id        
        positions = [ (p.position.x, p.position.y, p.position.z) for p in poses.poses]
        orientations = [ (p.orientation.x, p.orientation.y, p.orientation.z) for p in poses.poses]
        return self.__move_arm_trajectory_non_collision("which_arm",
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

    #def build_collision_operations(self, object1, object2):
        #msg = OrderedCollisionOperations()
        #collision = CollisionOperation()

        #collision.operation = CollisionOperation.DISABLE
        #collision.object1 = object1
        #collision.object2 = object2
        #msg.collision_operations.append(collision)
        #return msg
    
    def build_collision_operations(self, arm_name, 
                                   penetration_depth,
                                   enable=False):
        
        msg = OrderedCollisionOperations()
        
        
        if arm_name == "right_arm":
            link_names = ["r_gripper_palm_joint", 
                          "r_gripper_l_finger_joint", 
                          "r_gripper_l_finger_tip_joint",
                          "r_gripper_led_joint", 
                          "r_gripper_motor_accelerometer_joint", 
                          "r_gripper_motor_slider_joint",
                          "r_gripper_motor_screw_joint", 
                          "r_gripper_r_finger_joint",
                          "r_gripper_r_finger_tip_joint",
                          "r_gripper_joint", 
                          "r_gripper_tool_joint",
                          ]
        else:
            link_names = ["l_gripper_palm_joint", 
                          "l_gripper_l_finger_joint", 
                          "l_gripper_l_finger_tip_joint",
                          "l_gripper_led_joint", 
                          "l_gripper_motor_accelerometer_joint", 
                          "l_gripper_motor_slider_joint",
                          "l_gripper_motor_screw_joint", 
                          "l_gripper_r_finger_joint",
                          "l_gripper_r_finger_tip_joint",
                          "l_gripper_joint", 
                          "l_gripper_tool_joint",
                          ]           
        for link in link_names:
            collision = CollisionOperation()
            if enable:
                collision.operation = CollisionOperation.ENABLE
            else:
                collision.operation = CollisionOperation.DISABLE
            collision.object1 = link
            collision.object2 = "collision_map"
            collision.penetration_distance = penetration_depth
            msg.collision_operations.append(collision)            
        return msg    

    #def build_allowed_contact_specification(self, box_pose, box_dimensions):
        #msg = AllowedContactSpecification()
        #msg.name = "grasping_object_region"
        #shape = Shape()
        #shape.type = shape.BOX
        #shape.dimensions = box_dimensions

        #msg.shape = shape
        #msg.pose_stamped = box_pose

        #msg.link_names = ["r_gripper_palm_link",
                          #"r_gripper_l_finger_link",
                          #"r_gripper_r_finger_link",
                          #"r_gripper_l_finger_tip_link",
                          #"r_gripper_r_finger_tip_link",
                          #"l_gripper_palm_link",
                          #"l_gripper_l_finger_link",
                          #"l_gripper_r_finger_link",
                          #"l_gripper_l_finger_tip_link",
                          #"l_gripper_r_finger_tip_link"]
        #return msg

    def build_allowed_contact_specification(self,
                                            shape,
                                            pose,
                                            arm_name,                                            
                                            penetration_depth,
                                            collision_object_name = "collision_map",
                                            ):
        """
        Creates a motion_planning_msgs/AllowedContactSpecification message. This can be passed
        to a move_[left|right]_arm call to allow for contacts.
        
        Parameters:
        name: the name of the regions
        shape: the shape of the region in the environment. A arm_navigation_msgs/Shape msg
        pose: the PoseStamped of the space defining the region
        arm_name: either "right_arm" or "left_arm", the arm which is allowed to collide
        penetration_depth: the maximum penetration depth allowed for every link        
        """
        
        
        if arm_name == "right_arm":
            link_names = ["r_gripper_palm_joint", 
                          "r_gripper_l_finger_joint", 
                          "r_gripper_l_finger_tip_joint",
                          "r_gripper_led_joint", 
                          "r_gripper_motor_accelerometer_joint", 
                          "r_gripper_motor_slider_joint",
                          "r_gripper_motor_screw_joint", 
                          "r_gripper_r_finger_joint",
                          "r_gripper_r_finger_tip_joint",
                          "r_gripper_joint", 
                          "r_gripper_tool_joint",
                          ]
        else:
            link_names = ["l_gripper_palm_joint", 
                          "l_gripper_l_finger_joint", 
                          "l_gripper_l_finger_tip_joint",
                          "l_gripper_led_joint", 
                          "l_gripper_motor_accelerometer_joint", 
                          "l_gripper_motor_slider_joint",
                          "l_gripper_motor_screw_joint", 
                          "l_gripper_r_finger_joint",
                          "l_gripper_r_finger_tip_joint",
                          "l_gripper_joint", 
                          "l_gripper_tool_joint",
                          ]        

        collisions = []            
        for link in link_names:
            msg = AllowedContactSpecification()
            msg.name = "collision " + link + " vs " + collision_object_name    
            msg.shape = shape
            msg.pose_stamped = pose
            msg.link_names = [link, collision_object_name]                
            msg.penetration_depth = penetration_depth
            collisions.append(msg)
        return collisions

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

    def point_gripper_at(self, which_arm,
                         target,
                         diff_x = 0.2,
                         diff_y = 0.2,
                         diff_z = 0.3,
                         num_trials = 100,
                         visualize_arrow = False):
        """

        Parameters:
        target: a PoseStamped
        which_arm: either left_arm or right_arm
        """

        assert isinstance(target, PoseStamped)
        red_color = (1,1,0,0)
        green_color = (1,0,1,0)

        if target.header.frame_id != "/base_link":
            rospy.loginfo("Changing the frame from %s to %s", target.header.frame_id,
                          "/base_link")
            self.tf_listener.waitForTransform("/base_link", target.header.frame_id,
                                              rospy.Time.now(), rospy.Duration(1))
            target = self.tf_listener.transformPose("/base_link", target)

        tx = target.pose.position.x
        ty = target.pose.position.y
        tz = target.pose.position.z

        current_trial = 0
        if which_arm == "right_arm":
            ik_mover = self.move_right_arm_with_ik
            planner_mover = self.move_right_arm
        else:
            ik_mover = self.move_left_arm_with_ik
            planner_mover = self.move_left_arm

        while current_trial < num_trials:
            gripper_x = random.uniform(tx - diff_x, tx)
            gripper_y = random.uniform(ty - diff_y, ty + diff_y)
            gripper_z = random.uniform(tz, tz + diff_z)

            gripper_pose = (gripper_x, gripper_y, gripper_z)

            vec = (-gripper_x + tx,
                   -gripper_y + ty,
                   -gripper_z + tz,
                   )

            rot_mat = utils.make_orth_basis(vec)
            M = np.identity(4)
            M[:3, :3] = rot_mat
            gripper_orientation = tf.transformations.quaternion_from_matrix(M)

            #rospy.loginfo("Trying pose: %s, orientation: %s",
                          #gripper_pose, gripper_orientation)

            if visualize_arrow:
                arrow_pose = PoseStamped()
                arrow_pose.header.frame_id = "/base_link"
                arrow_pose.pose.position.x = gripper_pose[0]
                arrow_pose.pose.position.y = gripper_pose[1]
                arrow_pose.pose.position.z = gripper_pose[2]
                arrow_pose.pose.orientation.x = gripper_orientation[0]
                arrow_pose.pose.orientation.y = gripper_orientation[1]
                arrow_pose.pose.orientation.z = gripper_orientation[2]
                arrow_pose.pose.orientation.w = gripper_orientation[3]

            if planner_mover(gripper_pose,
                             gripper_orientation,
                             "/base_link",
                             10
                             ):
                rospy.loginfo("Pointing with planner was ok")  
                return True

            elif (current_trial>10) and  ik_mover(gripper_pose,
                        gripper_orientation,
                        "/base_link",
                        5):
                rospy.loginfo("Pointing was successful")
                if visualize_arrow:
                    self.__visualize_arrow(arrow_pose,
                                           target,
                                           current_trial,
                                           green_color, 0.01, 0.01)

                return True
            if visualize_arrow:
                self.__visualize_arrow(arrow_pose,
                                       target,
                                       current_trial,
                                       red_color, 0.01, 0.01)
            current_trial += 1

        return False


    def point_right_gripper_at(self, target,
                               diff_x = 0.2,
                               diff_y = 0.2,
                               diff_z = 0.3,
                               num_trials = 100):
        """

        Parameters:
        target: a PoseStamped
        """
        return self.point_gripper_at("right_arm",
                                     target,
                                     diff_x,
                                     diff_y,
                                     diff_z,
                                     num_trials
                                     )
    def point_left_gripper_at(self, target,
                              diff_x = 0.2,
                              diff_y = 0.2,
                              diff_z = 0.3,
                              num_trials = 100):
        """

        Parameters:
        target: a PoseStamped
        """
        return self.point_gripper_at("left_arm",
                                     target,
                                     diff_x,
                                     diff_y,
                                     diff_z,
                                     num_trials
                                     )


    def __visualize_arrow(self, tail_pose,
                          tip_pose,
                          marker_id,
                          color = (1.0, 1.0, 0, 0),
                          shaft_radius = 0.2,
                          head_radius = 0.01):
        assert isinstance(tail_pose, PoseStamped)
        assert isinstance(tip_pose, PoseStamped)
        arrow = Marker()

        arrow.points.append(tail_pose.pose.position)
        arrow.points.append(tip_pose.pose.position)

        arrow.header.frame_id = tail_pose.header.frame_id
        arrow.action = Marker.ADD
        arrow.type = Marker.ARROW
        arrow.lifetime = rospy.Duration(0)
        arrow.color.a = color[0]
        arrow.color.r = color[1]
        arrow.color.g = color[2]
        arrow.color.b = color[3]

        arrow.scale.x = shaft_radius
        arrow.scale.y = head_radius
        arrow.id = marker_id

        self.arrows_pub.publish(arrow)



if __name__ == "__main__":
    rospy.init_node('trytest', anonymous=True)
    move_arm = PR2MoveArm()

    position = (0.55 - 0.1, -0.188, 0)
    orientation = (0., 0., 0., 1.)

    if move_arm.move_right_arm(position, orientation, "/torso_lift_link", 120.):
        rospy.loginfo("OK")
    else:
        rospy.loginfo("bad")



