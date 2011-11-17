from xml.dom import minidom
import roslib
roslib.load_manifest("pr2_control_utilities")
import rospy
import actionlib
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import (JointTrajectoryAction,
                                      Pr2GripperCommandAction,
                                      SingleJointPositionAction,
                                      PointHeadAction)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class __RobotState(object):
    '''
    A class to keep track of all the joint positions, divided by category.
    
    '''
    
    left_joint_names = ('l_shoulder_pan_joint', 
                           'l_shoulder_lift_joint',
                           'l_upper_arm_roll_joint',
                           'l_elbow_flex_joint',
                           'l_forearm_roll_joint',
                           'l_wrist_flex_joint',
                           'l_wrist_roll_joint')
    right_joint_names = ('r_shoulder_pan_joint', 
                           'r_shoulder_lift_joint',
                           'r_upper_arm_roll_joint',
                           'r_elbow_flex_joint',
                           'r_forearm_roll_joint',
                           'r_wrist_flex_joint',
                           'r_wrist_roll_joint')
    
    l_gripper_names = ('l_gripper_joint',)        
    r_gripper_names = ('r_gripper_joint',)
    
    head_joint_names = ('head_pan_joint', 'head_tilt_joint')
    
    torso_joint_names = ("torso_lift_joint",)
        
    all = (left_joint_names + 
           right_joint_names + 
           head_joint_names + 
           l_gripper_names +  
           r_gripper_names +
           torso_joint_names
           )
    
    def __init__(self):
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback)
        
        self.left_arm_positions = []
        self.right_arm_positions = []
        self.head_positions = []
        self.l_gripper_positions = []
        self.r_gripper_positions = []
        self.torso_position = []
        self.joint_limits = {}
        
        self.r_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)        
        self.l_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.r_gripper_client = actionlib.SimpleActionClient("r_gripper_controller/gripper_action", Pr2GripperCommandAction)        
        self.l_gripper_client = actionlib.SimpleActionClient("l_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.torso_client = actionlib.SimpleActionClient("torso_controller/position_joint_action", SingleJointPositionAction)
        self.head_pointer_client = actionlib.SimpleActionClient("head_traj_controller/point_head_action", PointHeadAction)
        self.head_client = rospy.Publisher('/head_traj_controller/command', JointTrajectory, latch=True)
        
        # Some seemingly very important waits........
        rospy.loginfo("Waiting for joint trajectory actions")
        self.r_arm_client.wait_for_server()
        self.l_arm_client.wait_for_server()
        self.r_gripper_client.wait_for_server()
        self.l_gripper_client.wait_for_server()
        self.head_pointer_client.wait_for_server()
        self.torso_client.wait_for_server()
        
        self.r_arm_client.cancel_all_goals()
        self.l_arm_client.cancel_all_goals()
        self.r_gripper_client.cancel_all_goals()
        self.l_gripper_client.cancel_all_goals()
        self.head_pointer_client.cancel_all_goals()
        self.torso_client.cancel_all_goals()
        
        self.read_joint_limits()
        
        rospy.loginfo("Robot State is ready")
    
    def read_joint_limits(self):
        urdf = rospy.get_param("robot_description")
        try:
            urdf_doc = minidom.parseString(urdf)
        except IOError, e:
            rospy.loginfo("robot_description failed to parse. Are you sure it valid?\nError is: %s" %e)
            return
        
        joints = urdf_doc.getElementsByTagName('joint')
        for jnt in joints:
            name = jnt.attributes['name'].value
            
            lims = jnt.getElementsByTagName('limit')
            if len(lims) == 0:
                continue
    
            lower = lims[0].getAttribute('lower')
            upper = lims[0].getAttribute('upper')
    
            # Ignore joints with no limits
            if lower == '' or upper == '':
                continue
    
            self.joint_limits[name] = (float(lower), float(upper))
            
    def __getstate__(self):
        '''
        Used for pickle.
        '''
        return "bogus"
    
    def __setstate__(self, state):        
        '''
        Used for pickle.
        '''
        #state is bogus
        self.__init__()

    
    def joint_states_callback(self, msg):
        self.left_arm_positions = []
        for joint in self.left_joint_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
                return
            index = msg.name.index(joint)
            self.left_arm_positions.append(msg.position[index])
            
        self.right_arm_positions = []
        for joint in self.right_joint_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
                return
            index = msg.name.index(joint)
            self.right_arm_positions.append(msg.position[index])
            
        self.head_positions = []
        for joint in self.head_joint_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
                return
            index = msg.name.index(joint)
            self.head_positions.append(msg.position[index])
        
        self.l_gripper_positions = []
        for joint in self.l_gripper_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
                return
            index = msg.name.index(joint)
            self.l_gripper_positions.append(msg.position[index])    
        
        self.r_gripper_positions = []
        for joint in self.r_gripper_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
                return
            index = msg.name.index(joint)
            self.r_gripper_positions.append(msg.position[index])
            
        self.torso_position = []
        for joint in self.torso_joint_names:
            if not joint in msg.name:
                rospy.logerr("Error: joint %s is not in the joint_states!"%joint)
                return
            index = msg.name.index(joint)
            self.torso_position.append(msg.position[index])
            
__singe_instance = None

def RobotState():
    global __singe_instance
    if __singe_instance is None:
        __singe_instance = __RobotState()
    return __singe_instance
    
