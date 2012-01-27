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
import pr2_control_utilities

from interactive_markers.interactive_marker_server import (
        InteractiveMarkerServer, InteractiveMarker,
        Marker, InteractiveMarkerControl,
        )
from interactive_markers.menu_handler import MenuHandler

import copy

class MoveArmMarker(object):
    """
    This class holds an interactive marker for precise positioning of the
    PR2 arm. The marker uses arrows (MOVE_AXIS) for positioning. The interactive
    marker is published on the topic move_arm_marker.
    It relies on the IK and move_arm services and nodes to be active (see the
    documentation for pr2_control_utilities).

    The marker shows as a small red spehere initially positioned at (0,0,0)
    in the /base_link coordinates frame.
    """

    def __init__(self):        
        robot_state = pr2_control_utilities.RobotState()
        self.joint_controller = pr2_control_utilities.PR2JointMover(robot_state)
        self.planner = pr2_control_utilities.PR2MoveArm(self.joint_controller)
    
        self.server = InteractiveMarkerServer("move_arm_marker")

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/base_link"
        int_marker.name = "move_arm"
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

        #y movement
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
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

        #menu control
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "Menu"
        menu_control.description = "Commands"
        menu_control.markers.append(copy.deepcopy(marker))
        int_marker.controls.append(menu_control)

        self.server.insert(int_marker, self.main_callback)

        menu_handler = MenuHandler()
        menu_handler.insert("Move Head", callback=self.move_head)
        right_handle = menu_handler.insert( "Right Arm")
        left_handle = menu_handler.insert( "Left Arm")
        
        menu_handler.insert( "Planning", parent=right_handle, 
                callback=self.plan_right_arm )
        menu_handler.insert( "Collision Free", parent=right_handle, 
                callback=self.collision_free_right_arm )
        menu_handler.insert( "Planning", parent=left_handle, 
                callback=self.plan_left_arm )
        menu_handler.insert( "Collision Free", parent=left_handle, 
                callback=self.collision_free_left_arm )

        menu_handler.apply(self.server, int_marker.name)
        self.server.applyChanges()
        rospy.loginfo("MoveArmMarker ready")

    def main_callback(self, feedback):
        pass

    def move_head(self, feedback):
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )

        print "Moving the head"
        self.joint_controller.time_to_reach = 1.0
        self.joint_controller.point_head_to(pos, frame)

    def plan_right_arm(self, feedback):
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )
        orientation = (1,0,0,1)
       
        rospy.loginfo("Moving the right arm")
        self.planner.move_right_arm(pos, orientation, frame, 2.0)

    def collision_free_right_arm(self, feedback):
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )
        orientation = (1,0,0,1)
       
        rospy.loginfo("Moving the right arm (non collision)")
        self.planner.move_right_arm_non_collision(pos, orientation, frame, 2.0)

    def plan_left_arm(self, feedback):
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )
        orientation = (1,0,0,1)
       
        rospy.loginfo("Moving the left arm")
        self.planner.move_left_arm(pos, orientation, frame, 2.0)

    def collision_free_left_arm(self, feedback):
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )
        orientation = (1,0,0,1)
       
        rospy.loginfo("Moving the left arm (non collision)")
        self.planner.move_left_arm_non_collision(pos, orientation, frame, 2.0)


if __name__ == "__main__":
    rospy.init_node("move_arm_marker")
    server = MoveArmMarker()
    rospy.spin()
