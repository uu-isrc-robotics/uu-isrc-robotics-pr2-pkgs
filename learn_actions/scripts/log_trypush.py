#! /usr/bin/python
"""

"""

import roslib; 
roslib.load_manifest('learn_actions')
import rospy
import pr2_control_utilities
import tabletop_actions.object_detector as object_detector
import tabletop_actions.pushers as pushers

import learn_actions 

def main():
    robot_state = pr2_control_utilities.RobotState()
    joint_mover = pr2_control_utilities.PR2JointMover(robot_state)
    planner = pr2_control_utilities.PR2MoveArm(joint_mover)
    detector = object_detector.ObjectDetector()
    pusher_l = pushers.LeftArmLateralPusher(planner, robot_state)
    pusher_r = pushers.RightArmLateralPusher(planner, robot_state)
    base_mover = pr2_control_utilities.PR2BaseMover(planner.tf_listener,
                                                    use_controller = True,
                                                    use_move_base = False,
                                                    use_safety_dist = True 
                                                    )

    logger = learn_actions.LogPushingResult(detector, 
                                            pusher_l, 
                                            pusher_r,
                                            joint_mover,
                                            base_mover,
                                            tf_listener = planner.tf_listener)
    joyaction = learn_actions.JoyAction(3, logger.try_push_and_publish)

if __name__ == "__main__":
    rospy.init_node("log_trypush", anonymous=True)
    main()
    rospy.spin()
