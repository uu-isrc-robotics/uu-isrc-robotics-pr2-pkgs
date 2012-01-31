#!/usr/bin/env python
"""
Copyright (c) 2012, Lorenzo Riano.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""


import roslib; roslib.load_manifest("markers_tests")
import rospy
import copy
import pr2_control_utilities
import utils
from geometry_msgs.msg import PoseStamped

from interactive_markers.interactive_marker_server import (
        InteractiveMarkerServer, InteractiveMarker,
        Marker, InteractiveMarkerControl,
        )
from visualization_msgs.msg import Marker
from interactive_markers.menu_handler import MenuHandler

class TrackArmMarker(object):
    """
    This class holds an interactive marker that the PR2 arm will try to follow.
    The interactive marker is published on the topic track_arm_marker.
    It relies on the IK and move_arm services and nodes to be active (see the
    documentation for pr2_control_utilities).

    The marker shows as a small red spehere initially positioned at (0,0,0)
    in the /base_link coordinates frame.
    """

    def __init__(self):        
        robot_state = pr2_control_utilities.RobotState()
        self.joint_controller = pr2_control_utilities.PR2JointMover(robot_state)
        self.planner = pr2_control_utilities.PR2MoveArm(self.joint_controller)
        self.pose_publisher  = rospy.Publisher("/pezz_controller/command", 
                                               PoseStamped)
        self.track_right = False
        self.track_left = False
        self.track_head = False
    
        self.server = InteractiveMarkerServer("track_marker", q_size=1)

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/base_link"
        int_marker.name = "track_arm_marker"
        int_marker.description = "Move the arm"

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

        #x movement
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        int_marker.controls.append(control);

        #x rotation
        control = InteractiveMarkerControl()
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        int_marker.controls.append(control);

        #y movement
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        int_marker.controls.append(control);

        #y rotation
        control = InteractiveMarkerControl()
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        int_marker.controls.append(control);

        #z movement
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        int_marker.controls.append(control);

        #z rotation
        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        int_marker.controls.append(control);

        #menu control
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "Menu"
        menu_control.description = "Commands"
        menu_control.markers.append(copy.deepcopy(marker))
        int_marker.controls.append(menu_control)

        self.server.insert(int_marker, self.main_callback)

        menu_handler = MenuHandler()
        head_handle = menu_handler.insert("Track Head", 
                callback=self.toggle_head)
        right_handle = menu_handler.insert( "Track Right Arm",
            callback = self.toggle_right)
        left_handle = menu_handler.insert( "Track Left Arm",
            callback = self.toggle_left)

        menu_handler.insert("Place at right gripper",
                callback = self.place_right_gripper)
        menu_handler.insert("Place at left gripper",
                callback = self.place_left_gripper)

        menu_handler.setCheckState(right_handle, MenuHandler.UNCHECKED)
        menu_handler.setCheckState(left_handle, MenuHandler.UNCHECKED)
        menu_handler.setCheckState(head_handle, MenuHandler.UNCHECKED)
        
        menu_handler.apply(self.server, int_marker.name)
        
        self.menu_handler = menu_handler
        self.right_handle = right_handle
        self.left_handle = left_handle
        self.head_handle = head_handle
        self.int_marker = int_marker

        self.server.applyChanges()
        rospy.loginfo("MoveArmMarker ready")

    def main_callback(self, feedback):
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


        self.joint_controller.time_to_reach = 0.1
        if self.track_head:
            self.joint_controller.point_head_to(pos, frame)

        posmsg = PoseStamped()
        posmsg.header.frame_id = frame
        posmsg.pose = feedback.pose
        if self.track_right:
            self.pose_publisher.publish(posmsg)    
	
#            self.planner.move_right_arm_non_collision(pos, 
#                               orientation, frame, 0.0)
        
#        if self.track_left:
#            self.planner.move_left_arm_non_collision(pos, 
#                                orientation, frame, 0.0)

    def toggle_head(self, feedback):
        if self.track_head:
            rospy.loginfo("Stopping head tracking")
            self.track_head = False
            self.menu_handler.setCheckState(self.head_handle, 
                    MenuHandler.UNCHECKED)
        else:
            rospy.loginfo("Tracking with head")
            self.track_head = True
            self.menu_handler.setCheckState(self.head_handle, 
                    MenuHandler.CHECKED)


    def toggle_right(self, feedbakck):
        if self.track_right:
            self.track_right = False
            self.menu_handler.setCheckState(self.right_handle, 
                    MenuHandler.UNCHECKED)
            rospy.loginfo("Stopping track with the right arm")
        else:
            self.track_right = True
            self.menu_handler.setCheckState(self.right_handle, 
                    MenuHandler.CHECKED)
            self.track_left = False
            self.menu_handler.setCheckState(self.left_handle, 
                    MenuHandler.UNCHECKED)
            rospy.loginfo("Tracking with the right arm")
        
        self.menu_handler.apply(self.server, self.int_marker.name)
        self.server.applyChanges()

    def toggle_left(self, feedbakck):
        if self.track_left:
            self.track_left = False
            self.menu_handler.setCheckState(self.left_handle, 
                    MenuHandler.UNCHECKED)
            rospy.loginfo("Stopping track with the left arm")
        else:
            self.track_left = True
            self.menu_handler.setCheckState(self.left_handle, 
                    MenuHandler.CHECKED)
            self.track_right = False
            self.menu_handler.setCheckState(self.right_handle, 
                    MenuHandler.UNCHECKED)
            rospy.loginfo("Tracking with the left arm")
        
        self.menu_handler.apply(self.server, self.int_marker.name)
        self.server.applyChanges()

    def place_right_gripper(self, feedback):
        gripper_pos = self.planner.get_right_gripper_pose()
        self.server.setPose(self.int_marker.name, gripper_pos.pose, 
                gripper_pos.header)
        self.server.applyChanges()
    
    def place_left_gripper(self, feedback):
        gripper_pos = self.planner.get_left_gripper_pose()
        self.server.setPose(self.int_marker.name, gripper_pos.pose, 
                gripper_pos.header)
        self.server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("track_arm_marker")
    server = TrackArmMarker()
    rospy.spin()
