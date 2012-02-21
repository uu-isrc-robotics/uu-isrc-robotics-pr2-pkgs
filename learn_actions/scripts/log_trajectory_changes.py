#! /usr/bin/python
"""

"""

import roslib; 
roslib.load_manifest('learn_actions')
import rospy
import pr2_control_utilities
import tabletop_actions.object_detector as object_detector

import learn_actions 

def main():
    robot_state = pr2_control_utilities.RobotState()
    joint_mover = pr2_control_utilities.PR2JointMover(robot_state)
    planner = pr2_control_utilities.PR2MoveArm(joint_mover)
    detector = object_detector.ObjectDetector()
    
    arm = "left"
    #arm = "right"
    service_name = "/pr2_trajectory_markers_" + arm + "/execute_trajectory"
    logger = learn_actions.LogTrajectoryResult(detector, 
                                            joint_mover,
                                            service_name,
                                            arm,
                                            planner,
                                            tf_listener = planner.tf_listener)
    raw_input("press a key to start...")
    logger.publish_object_changes()

if __name__ == "__main__":
    rospy.init_node("log_changes", anonymous=True)
    main()
    rospy.loginfo("Done")
