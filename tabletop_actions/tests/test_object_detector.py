#! /usr/bin/python

PKG="tabletop_explorer"

import roslib
roslib.load_manifest(PKG)

import rospy
import pr2_control_utilities
import tabletop_actions.object_detector as object_detector 

if __name__ == "__main__":
    rospy.init_node(PKG, anonymous=True)
    
    robot_state = pr2_control_utilities.RobotState()
    joint_mover = pr2_control_utilities.PR2JointMover(robot_state)
    detector = object_detector.ObjectDetector()
    
    if not detector.search_for_object(joint_mover, trials=15,
                                      cluster_choser="find_closest_cluster",
                                      max_pan=0.5, min_pan=-0.5,
                                      max_tilt = 1.1, min_tilt = 0.6,):
        rospy.logerr("No object detected by the stereo")
    else:
        rospy.loginfo("Object detected")
        rospy.loginfo("Table message:\n%s", detector.last_detection_msg.detection.table)
    
    