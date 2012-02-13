#! /usr/bin/env python

import roslib; roslib.load_manifest("pr2_control_utilities")
import rospy
from pr2_control_utilities.pr2_joint_mover import PR2JointMover

if __name__=="__main__":
    rospy.init_node("pr2_joint_mover_use_example")
    jmover = PR2JointMover()
    print "WARNING: Make sure there is space around your PR2"
    raw_input("Press Enter to continue...")
    
    # Populate the one you want to move, e.g. if you want to move all
    jmover.target_head = [0, 0]
    jmover.target_torso = 0.12
    jmover.target_left_gripper = 0
    jmover.target_right_gripper = 0
    jmover.target_left_arm = [0, 0, 0, 0, 0, 0, 0]
    jmover.target_right_arm = [0, 0, 0, 0, 0, 0, 0]
    # and finally call execute
    jmover.execute()
    # or if you want pr2 to block and wait for action to finish, then call
    #jmover.execute_and_wait()
    pass

