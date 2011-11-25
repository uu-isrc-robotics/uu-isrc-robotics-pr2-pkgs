#! /usr/bin/python
import roslib 
roslib.load_manifest('learn_actions')
import rospy
from learn_actions.msg import ObjectDiscovery
from learn_actions import approach_fsm, joystick_action
import pr2_control_utilities
import tabletop_actions.pushers as pushers
import tabletop_actions.object_detector as object_detector
import functools

class FSMLogger(object):
    """
    Executes an FSM and logs the result.

    It first invokes the object detector to log the execution conditions,
    then it executes the FSM and it records the result.
    The resulting message is then published.
    """

    def __init__(self, detector, fsm_creator, joint_mover):
        self.detector = detector
        self.fsm_creator = fsm_creator
        self.joint_mover = joint_mover
        self.obj_discovery_pub = rospy.Publisher("object_discovery", 
                ObjectDiscovery)

    def publish_result(self, box, table, whicharm, torso_joint):
        
        msg = ObjectDiscovery()
        msg.header.stamp = rospy.Time.now()

        msg.object_pose.x = box.pose.pose.position.x
        msg.object_pose.y = box.pose.pose.position.y
        msg.object_pose.z = box.pose.pose.position.z
        
        msg.table = table
        
        msg.grasping_result = whicharm
        msg.torso_joint = torso_joint
        
        self.obj_discovery_pub.publish(msg) 

    def execute(self):
        """
        Records the object position, executes the FSM and published the result.
        
        """
        fsm = self.fsm_creator()
        if not self.detector.search_for_object(self.joint_mover, trials=15,  
                                      cluster_choser="find_closest_cluster", 
                                      max_pan=0.5, min_pan=-0.5,
                                      max_tilt = 1.1, min_tilt = 0.75,):
            rospy.logerr("No object detected")
            return
        box = self.detector.last_box_msg
        table = self.detector.last_detection_msg.detection.table
        obj_pos =  (box.pose.pose.position.x,
                    box.pose.pose.position.y,
                    box.pose.pose.position.z)
        fsm.userdata.obj_pos = obj_pos
        
        rospy.loginfo("Executing the FSM")
        res = fsm.execute()
        #TODO add torso_joint
        if res == "failure":
            self.publish_result(box, table, ObjectDiscovery.FAIL, 0)
        elif res == "l_arm":
            self.publish_result(box, table, ObjectDiscovery.LEFT_ARM, 0)
        else:
            self.publish_result(box, table, ObjectDiscovery.RIGHT_ARM, 0)

            
def main():
    """
    Register the FSMLogger with the JoyCallback.
    """
    rospy.init_node("log_trypush", anonymous=True)
    robot_state = pr2_control_utilities.RobotState()
    joint_mover = pr2_control_utilities.PR2JointMover(robot_state)
    planner = pr2_control_utilities.PR2MoveArm(joint_mover)
    detector = object_detector.ObjectDetector()
    base_mover = pr2_control_utilities.PR2BaseMover(planner.tf_listener,
                                                    use_controller = True,
                                                    use_move_base = False,
                                                    use_safety_dist = True,
                                                    base_controller_name = "/client/cmd_vel",
                                                    )
    pusher_l = pushers.LeftArmLateralPusher(planner, robot_state)
    pusher_r = pushers.RightArmLateralPusher(planner, robot_state)
    obj_discovery_pub = rospy.Publisher("object_discovery", ObjectDiscovery)

    fsm_factory = functools.partial(approach_fsm.create_fsm,
                        base_mover, detector, 
                        joint_mover, pusher_r, 
                        pusher_l, obj_discovery_pub)
   
    fsm_logger = FSMLogger(detector, fsm_factory, joint_mover)
    joyaction = joystick_action.JoyAction(3, fsm_logger.execute)

    rospy.spin()



if __name__ == "__main__":
    main()
