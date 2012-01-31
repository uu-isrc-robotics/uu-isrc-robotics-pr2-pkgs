#!/usr/bin/env python
"""
Copyright (c) 2012, Lorenzo Riano.
All rights reserved.
"""


import roslib; roslib.load_manifest("markers_tests")
import rospy
import pr2_control_utilities
from pr2_control_utilities.utils import create_tuples_from_pose

from interactive_markers.interactive_marker_server import (
        InteractiveMarkerServer, InteractiveMarker,
        InteractiveMarkerControl,
        )
from interactive_markers.menu_handler import MenuHandler

import utils
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class TrajectoryMarkers(object):
    """
    A class to create and store a  trajectory for one PR2 arm. The created
    trajectory can be published as a PoseArray message.

    Constructor:
    TrajectoryMarkers(whicharm = "left")
    or
    TrajectoryMarkers(whicharm = "right")
    """
    def __init__(self, whicharm):
        self.whicharm = whicharm
        robot_state = pr2_control_utilities.RobotState()
        self.joint_controller = pr2_control_utilities.PR2JointMover(robot_state)
        self.planner = pr2_control_utilities.PR2MoveArm(self.joint_controller)
        self.server = InteractiveMarkerServer("trajectory_markers_" + whicharm)

        self.visualizer_pub = rospy.Publisher("trajectory_markers_path",
                MarkerArray)
        
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/base_link"
        int_marker.name = "move_" + whicharm + "_arm"
        int_marker.description = "Move the " + whicharm + " arm"
        self.server.insert(int_marker, self.main_callback)

        # create the main marker shape
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # create a non-interactive control which contains the sphere
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.markers.append( marker )
        int_marker.controls.append( control )
        
        utils.make_6DOF_marker(int_marker)
        self.int_marker = int_marker
        self.create_menu()
        self.server.applyChanges()

        self.trajectory = PoseArray()
        self.trajectory.header.frame_id = "/base_link"


    def create_menu(self):
        """
        Create and populates all the menu entries
        """
        menu_handler = MenuHandler()
        menu_handler.insert("Add position to trajectory", 
                callback = self.add_point)
        menu_handler.insert("Place marker over gripper", 
                callback = self.place_gripper)
        menu_handler.insert("Execute trjectory", 
                callback = self.execute_trajectory)
        menu_handler.insert("Clear trajectory", 
                callback = self.clear_trajectory)
        menu_handler.insert("Point the head", 
                callback = self.move_head)
        menu_handler.insert("Move the arm (planning)", 
                callback = self.plan_arm)
        menu_handler.insert("Move the arm (collision-free)", 
                callback = self.collision_free_arm)
        menu_handler.insert("Update planning scene", 
                callback = self.update_planning)

        menu_handler.apply(self.server, self.int_marker.name)

    def main_callback(self, feedback):
        """
        empty
        """
        pass

    def add_point(self, feedback):
        """
        Add a point to self.trajectory if it is allowed by IK.
        """
        pos = PoseStamped()
        pos.header.frame_id = feedback.header.frame_id
        pos.pose = feedback.pose
        if self.whicharm == "right":
            ik = self.planner.check_ik_right_arm
        else:
            ik = self.planner.check_ik_left_arm
        
        if ik(pos):
            rospy.loginfo("Pose is reachable")
            self.trajectory.poses.append(feedback.pose)
        else:
            rospy.logerr("Pose is not reachable!")

    def place_gripper(self, feedback):
        """
        Move the marker where the gripper is
        """
        if self.whicharm == "right":
            gripper_pos = self.planner.get_right_gripper_pose()
        else:
            gripper_pos = self.planner.get_left_gripper_pose()
        self.server.setPose(self.int_marker.name, gripper_pos.pose, 
                gripper_pos.header)
        self.server.applyChanges()

    def execute_trajectory(self, feedback):
        """
        Executes the tracjectory memorized so far.
        """
        if self.whicharm == "right":
            moveit = self.planner.move_right_arm_non_collision
        else:
            moveit = self.planner.move_left_arm_non_collision

        for pose in self.trajectory.poses:
            pos, quat = create_tuples_from_pose(pose)
            res = moveit(pos, quat, self.trajectory.header.frame_id, 1.0)
            if not res:
                rospy.logerr("Something went wrong when moving")
                return

    def clear_trajectory(self, feedback):
        """
        Removes all the points stored so far
        """
        self.trajectory.poses = []

    def move_head(self, feedback):
        """
        Moves the head to face the marker
        """
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )

        print "Moving the head"
        self.joint_controller.time_to_reach = 1.0
        self.joint_controller.point_head_to(pos, frame)

    def plan_arm(self, feedback):
        """
        Moves the arm on the marker using motion collision-aware motion 
        planning.
        """
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )
        orientation = (feedback.pose.orientation.x,
               feedback.pose.orientation.y,
               feedback.pose.orientation.z,
               feedback.pose.orientation.w,
              )
      
        if self.whicharm == "right":
            rospy.loginfo("Moving the right arm")
            self.planner.move_right_arm(pos, orientation, frame, 2.0)
        else:
            rospy.loginfo("Moving the left arm")
            self.planner.move_left_arm(pos, orientation, frame, 2.0)

    def collision_free_arm(self, feedback):
        """
        Moves the rm on the marker using motion NON-collision-aware inverse
        kinematiks.
        """
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )
        orientation = (feedback.pose.orientation.x,
               feedback.pose.orientation.y,
               feedback.pose.orientation.z,
               feedback.pose.orientation.w,
              )
       
        if self.whicharm == "right":
            rospy.loginfo("Moving the right arm (non collision)")
            self.planner.move_right_arm_non_collision(pos, orientation, 
                                                      frame, 2.0)
        else:
            rospy.loginfo("Moving the left arm (non collision)")
            self.planner.move_left_arm_non_collision(pos, orientation, 
                                                     frame, 2.0)
    def update_planning(self, feedback):
        """
        Updates the planning scene.
        """
        self.planner.update_planning_scene()

    def publish_trajectory(self):
        """
        Publishes markers to visualize the current trajectory.
        """
        if len(self.trajectory.poses) == 0:
            return
        msg = MarkerArray()
        marker_id = 0
        
        #creating the path connecting the axes
        path = Marker()
        path.header.frame_id = self.trajectory.header.frame_id
        path.ns = "path"
        path.action = Marker.ADD
        path.type = Marker.LINE_STRIP
        path.lifetime = rospy.Duration(1/5.)
        path.color.r = 1
        path.color.g = 0
        path.color.b = 1
        path.color.a = 1
        path.scale.x = 0.01
        path.id = marker_id

        marker_id += 1
        for pose in self.trajectory.poses:
            pos = PoseStamped()
            pos.header.frame_id = self.trajectory.header.frame_id
            pos.pose = pose
            
            markers = utils.axis_marker(pos, marker_id, "axes")
            msg.markers.extend(markers)

            path.points.append(pose.position)

            marker_id += 3 #3 axes 
        
        msg.markers.append(path)
        self.visualizer_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("trajectory_markers")
    server = TrajectoryMarkers("left")
    t = rospy.Rate(5)
    while not rospy.is_shutdown():
        server.publish_trajectory()
        t.sleep()
