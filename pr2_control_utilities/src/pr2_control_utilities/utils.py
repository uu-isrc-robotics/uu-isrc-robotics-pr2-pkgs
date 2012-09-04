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
import math
import numpy as np

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

def create_tuples_from_pose(pose):
    """ Creates two tuples position (x,y,z) and rotation (x,y,z,w)
    from a geometry_msgs/Pose
    """
    pos = (pose.position.x,
               pose.position.y,
               pose.position.z,
              )
    orientation = (pose.orientation.x,
               pose.orientation.y,
               pose.orientation.z,
               pose.orientation.w,
              )
    return pos, orientation


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
                                        rospy.Time(0), rospy.Duration(5))
    zeropose = PointStamped()
    zeropose.header.frame_id = from_frame
    zeropose.point.x = pos[0]
    zeropose.point.y = pos[1]
    zeropose.point.z = pos[2]

    newpose = listener.transformPoint(to_frame, zeropose)
    return newpose.point.x, newpose.point.y, newpose.point.z

def normalize_trajectory(trajectory, current_angles):
    """normalize a trajectory (list of lists of joint angles),
    so that the desired angles are the nearest ones for the continuous
    joints (5 and 7)
    """
    trajectory_copy = [list(angles) for angles in trajectory]
    for angles in trajectory_copy:
        angles[4] = normalize_angle(angles[4], current_angles[4])
        angles[6] = normalize_angle(angles[6], current_angles[6])
    return trajectory_copy

def normalize_angle(angle, current_angle):
    """normalize an angle for a continuous joint so that it's the closest
    version of the angle to the current angle (not +-2*pi)
    """
    while current_angle-angle > math.pi:
        angle += 2*math.pi
    while angle - current_angle > math.pi:
        angle -= 2*math.pi
    return angle


def make_orth_basis(x_ax):
    """
    John Schulman magic code.
    """
    x_ax = np.asarray(x_ax)

    x_ax = x_ax / np.linalg.norm(x_ax)
    if np.allclose(x_ax, [1,0,0]):
        return np.eye(3)
    elif np.allclose(x_ax, [-1, 0, 0]):
        return np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, 1]])
    else:
        y_ax = np.r_[0, x_ax[2], -x_ax[1]]
        y_ax /= np.linalg.norm(y_ax)
        z_ax = np.cross(x_ax, y_ax)
        return np.c_[x_ax, y_ax, z_ax]