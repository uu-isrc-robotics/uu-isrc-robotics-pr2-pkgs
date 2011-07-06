#! /usr/bin/python
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)

import rospy
import tabletop_actions.object_detector as object_detector
import tabletop_actions.object_pickup as object_pickup
import pr2_control_utilities

import sys

def getout(msg):
    rospy.logerr(msg)
    sys.exit()    

def main():
    robot_state = pr2_control_utilities.RobotState()
    mover = pr2_control_utilities.PR2JointMover(robot_state)
    detector = object_detector.ObjectDetector()
    grabber = object_pickup.Grabber()
    
    if not detector.search_for_object(mover, trials=15,  use_random=True, 
                                      max_pan=0.5, min_pan=-0.5,
                                      max_tilt = 1.1, min_tilt = 0.75,):    
        getout("No object detected by the wide stereo")
    
    grabber.pickup_object(detector.last_collision_processing_msg, 
                          "right_arm", index = 0,
                          min_approach_distance=0.05,
                          lift_desired_distance=0.1)
    
    
if __name__ == "__main__":
    rospy.init_node(PKG, anonymous=True)
    main()