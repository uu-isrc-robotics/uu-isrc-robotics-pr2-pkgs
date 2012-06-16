#! /usr/bin/env python
# Copyright (c) 2010, Lorenzo Riano.


import roslib
roslib.load_manifest("pr2_control_utilities")
import rospy
from pr2_control_utilities.pr2_planning import PR2MoveArm
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":

    rospy.init_node('test_point_gripper', anonymous=True)
    mover = PR2MoveArm()

    point = PoseStamped()
    point.header.frame_id = "/base_link"
    point.pose.position.x = 0.347548782989
    point.pose.position.y = -0.217754007212
    point.pose.position.z = 0.730943471193

    mover.point_right_gripper_at(point)
