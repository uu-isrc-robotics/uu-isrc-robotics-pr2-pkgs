#! /usr/bin/python
"""
Basic finite state machine to search for an object, and approach it.
The robot aligns with the object and move towards the edge of the table,
until it stops.
"""

import roslib; 
roslib.load_manifest('learn_actions')
import smach
import rospy
import pr2_control_utilities
import tabletop_actions.object_detector as object_detector
import tabletop_actions.pushers as pushers
from learn_actions.msg import ObjectDiscovery


smach.set_loggers(rospy.loginfo,
                  rospy.logwarn,
                  lambda s: None,
                  rospy.logerr)
                
def publish_result(publisher, box, table, whicharm, torso_joint):
    
    msg = ObjectDiscovery()
    msg.header.stamp = rospy.Time.now()

    msg.object_pose.x = box.pose.pose.position.x
    msg.object_pose.y = box.pose.pose.position.y
    msg.object_pose.z = box.pose.pose.position.z
    
    msg.table = table
    
    msg.grasping_result = whicharm
    msg.torso_joint = torso_joint
    
    publisher.publish(msg) 



class ApproachTable(smach.State):
    """
    Approaches a table.

    The dimensions of the table can be passed through the
    userdata. Otherwise it will use the detector to find
    an object or the data.
    """
    def __init__(self, base_mover, detector, joint_mover):
        smach.State.__init__(self,
                             outcomes = ["success", "failure"],
                             input_keys =  ["table_dims",
                                            "obj_pos"
                                            ],
                             output_keys = ["table_dims",
                                            "obj_pos"]
                             )
        self.base_mover = base_mover
        self.detector = detector
        self.joint_mover = joint_mover
    
    def detect(self):
        if not self.detector.search_for_object(self.joint_mover, trials=15,  
                                      cluster_choser="find_closest_cluster", 
                                      max_pan=0.5, min_pan=-0.5,
                                      max_tilt = 1.1, min_tilt = 0.75,):
            rospy.logerr("No object detected")
            return None
        
        box = self.detector.last_box_msg
        table = self.detector.last_detection_msg.detection.table
        table_dims = (table.x_min,
                      table.x_max,
                      table.y_min,
                      table.y_max)
   
        obj_pos =  (box.pose.pose.position.x,
                    box.pose.pose.position.y,
                    box.pose.pose.position.z)

        return table_dims, obj_pos
    
    def execute(self, userdata):
        rospy.loginfo("Executing state ApproachTable")

        try:
            table_dims = userdata.table_dims
            obj_pos = userdata.obj_pos
        except KeyError:
            rospy.loginfo("No table given, going with the detector")
            ret = self.detect()
            if ret is None:
                return "failure"
            table_dims, obj_pos = ret

        dx = table_dims[0] #xmin
        dy = obj_pos[1] #y
        
        rospy.loginfo("Moving to %s", (dx,dy))

        ret = self.base_mover.drive_to_displacement((dx, dy, 0),
                                    safety_dist = 0.15
                                    )
        userdata.obj_pos = (obj_pos[0] - dx,
                            obj_pos[1] - dy,
                            obj_pos[2])
        userdata.table_dims = (table_dims[0] - dx,
                               table_dims[1] - dx,
                               table_dims[2] - dy,
                               table_dims[3] - dy,
                              )

        if ret:
            return "success"
        else:
            return "failure"

class TryToPush(smach.State):
    def __init__(self, pusher_r, pusher_l,detector, joint_mover, pub):
        smach.State.__init__(self,
                             outcomes = ["l_arm", "r_arm", "failure"],
                             input_keys =  ["obj_pos"
                                            ],
                             output_keys = []
                             )
 
        self.pusher_r = pusher_r
        self.pusher_l = pusher_l
        self.detector = detector
        self.joint_mover = joint_mover
        self.publisher = pub

    def detect(self, obj_pos=None):
        if obj_pos is not None:
            self.joint_mover.point_head_to(obj_pos, "base_link")
        if not self.detector.search_for_object(self.joint_mover, trials=15,  
                                      cluster_choser="find_closest_cluster", 
                                      max_pan=0.5, min_pan=-0.5,
                                      max_tilt = 1.1, min_tilt = 0.75,):
            rospy.logerr("No object detected")
            return None
        
        box = self.detector.last_box_msg
                      
        return box
 
    def execute(self, userdata):
        rospy.loginfo("Executing state TryToPush")
        
        try:
           obj_pos = userdata.obj_pos
        except KeyError:
            rospy.loginfo("no known object position")
            obj_pos = None
        
        box = self.detect(obj_pos)
        table = self.detector.last_detection_msg.detection.table
        
        if box is None:
            return "failure"

        rospy.loginfo("Trying the left pusher")
        ret =  self.pusher_l.test_push(box)
        if ret is not None:
            traj_pos, traj_angles = ret
            rospy.loginfo("Pushing with left arm is ok")
            return "l_arm"
        else: 
            rospy.loginfo("Trying the right pusher")
            ret = self.pusher_r.test_push(box)
            if ret is not None:
                rospy.loginfo("Pushing with right arm is ok")
                return "r_arm"
            else:
                rospy.logerr("Pushing is not feasible")
                return "failure" 

def create_fsm(base_mover, detector, joint_mover, pusher_r, pusher_l, pub):
    sm =  smach.StateMachine(outcomes=["l_arm", "r_arm", "failure"])

    with sm:
        smach.StateMachine.add("TryToPush1",  TryToPush(pusher_r, pusher_l,
                                                    detector, joint_mover,
                                                    pub),
                               transitions = {"l_arm" : "l_arm",
                                              "r_arm" : "r_arm",
                                             "failure" : "ApproachTable"},
                               remapping =  {"obj_pos" : "obj_pos"}
                              )
        smach.StateMachine.add("ApproachTable", ApproachTable(base_mover,
                                                 detector, joint_mover),
                               transitions = {"success" : "TryToPush2",
                                              "failure" : "TryToPush2"},
                               remapping = {"table_dims": "table_dims",
                                            "obj_pos" : "obj_pos"}
                               )
        smach.StateMachine.add("TryToPush2",  TryToPush(pusher_r, pusher_l,
                                                    detector, joint_mover,
                                                    pub),
                               transitions = {"l_arm" : "l_arm",
                                              "r_arm" : "r_arm",
                                             "failure" : "failure"},
                               remapping =  {"obj_pos" : "obj_pos"}
                              )
        return sm 

def test():
    robot_state = pr2_control_utilities.RobotState()
    joint_mover = pr2_control_utilities.PR2JointMover(robot_state)
    planner = pr2_control_utilities.PR2MoveArm(joint_mover)
    detector = object_detector.ObjectDetector()
    base_mover = pr2_control_utilities.PR2BaseMover(planner.tf_listener,
                                                    use_controller = True,
                                                    use_move_base = False,
                                                    use_safety_dist = True 
                                                    )
    pusher_l = pushers.LeftArmLateralPusher(planner, robot_state)
    pusher_r = pushers.RightArmLateralPusher(planner, robot_state)
    obj_discovery_pub = rospy.Publisher("object_discovery", ObjectDiscovery)

    fsm = create_fsm(base_mover, detector, joint_mover, pusher_r, pusher_l, obj_discovery_pub)
    
    fsm.execute()
        
if __name__ == "__main__":
    rospy.init_node("simple_approach_table", anonymous=True)
    test()
    rospy.spin()
