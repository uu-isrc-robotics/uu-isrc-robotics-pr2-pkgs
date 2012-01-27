'''
File: utils.py
Author: Lorenzo Riano 
Description: 
'''
import roslib
roslib.load_manifest("pr2_control_utilities")
import tf
import rospy
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import PointStamped


def convert_to_posestamped(listener, pos, rot, from_frame, to_frame):
    """
    Create a PoseStamped fomr position,rotation in a new frame

    @param pos: (x,y,z)
    @param rot: (th_x, th_y, th_z) euler angles
    @param listener: a TransformListener
    @return: a PoseStamped in the to_frame reference
    
    """
    listener.waitForTransform(to_frame, from_frame,
                                   rospy.Time(0), rospy.Duration(1))
    zeropose = PoseStamped()
    zeropose.header.frame_id = from_frame
    zeropose.pose.position.x = pos[0]
    zeropose.pose.position.y = pos[1]
    zeropose.pose.position.z = pos[2]
    quaternion = tf.transformations.quaternion_from_euler(rot[0], 
            rot[1], rot[2], axes="rxyz")
    zeropose.pose.orientation.x = quaternion[0]
    zeropose.pose.orientation.y = quaternion[1]
    zeropose.pose.orientation.z = quaternion[2]
    zeropose.pose.orientation.w = quaternion[3]
        
    newpose = listener.transformPose(to_frame, zeropose)
    return newpose

def convert_to_pos_rot(pos):
    """Convert a PoseStamped to two tuples (x,y,z) and 3 euler angles"""
    pose = (pos.pose.position.x,
           pos.pose.position.y,
           pos.pose.position.z)
    
    quaternion = (pos.pose.orientation.x,
                  pos.pose.orientation.y,
                  pos.pose.orientation.z,
                  pos.pose.orientation.w)
    rot = tf.transformations.euler_from_quaternion(quaternion,axes="rxyz")
    
    return pose, rot
    
    pass



def convert_position(listener, pos, rot, from_frame, to_frame):
    """
    Convert position, rotation between frames

    @param pos: (x,y,z)
    @param rot: (th_x, th_y, th_z) euler angles
    @param listener: a TransformListener
    @return: the two tuples in the to_frame reference
    
    """
    newpose = convert_to_posestamped(listener, pos, rot, from_frame, to_frame)
    trans = (newpose.pose.position.x,
            newpose.pose.position.y,
            newpose.pose.position.z)
    quaternion = (newpose.pose.orientation.x,
                newpose.pose.orientation.y,
                newpose.pose.orientation.z,
                newpose.pose.orientation.w)
    rot = tf.transformations.euler_from_quaternion(quaternion,axes="rxyz")
    return trans,rot
 
def convert_point(listener, pos, from_frame, to_frame):
    """
    Create a PointStamped from position in a new frame

    @param pos: (x,y,z)
    @param listener: a TransformListener
    @return: a PointStamped in the to_frame reference
    
    """
    listener.waitForTransform(to_frame, from_frame,
                                        rospy.Time(0), rospy.Duration(1))
    zeropose = PointStamped()
    zeropose.header.frame_id = from_frame
    zeropose.point.x = pos[0]
    zeropose.point.y = pos[1]
    zeropose.point.z = pos[2]
        
    newpose = listener.transformPoint(to_frame, zeropose)
    return newpose.point.x, newpose.point.y, newpose.point.z

