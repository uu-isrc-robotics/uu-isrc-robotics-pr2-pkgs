#! /usr/bin/python

import roslib
roslib.load_manifest('learn_actions')
import rospy
import tf

from pr2_control_utilities import RobotState
from pr2_control_utilities import PR2MoveArm
from tabletop_actions.object_detector import GenericDetector
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessingResponse
from geometry_msgs.msg import PoseStamped
from learn_actions.msg import ObjectAndJoints
import tf

def search_object(tf_listener):
    """
    Searches for objects using the detector.

    Returns:
    a list of (pos, dims, names) where pos is a PoseStamped(converted in the self.frame fame with
    the object pose if the object is found), dims is a Vector3 specifing the x,y,z dimensions of
    the box and names are the names of the objects.

    None if no object is found
    """
    detector = GenericDetector()
    res = detector.detect()

    if not res:
        return None

    coll_res = detector.call_collision_map_processing(res)

    isinstance(coll_res, TabletopCollisionMapProcessingResponse)

    rospy.loginfo("%d objects found, changing their poses into frame %s",
                  len(coll_res.graspable_objects),frame)

    poses = []
    dims = []

    for graspable in coll_res.graspable_objects:
        cluster = graspable.cluster
        box_pose = detector.detect_bounding_box(cluster).pose
        tf_listener.waitForTransform(frame,
                                     box_pose.header.frame_id,
                                     rospy.Time(0),
                                     rospy.Duration(1)
                                     )
        object_pose = tf_listener.transformPose(frame, box_pose)
        poses.append(object_pose)
        dims.append(detector.last_box_msg.box_dims)

    return poses, dims, coll_res.collision_object_names

def start_log(tf_listener):

    isinstance(tf_listener, tf.TransformListener)
    obj_changes_pub = rospy.Publisher("object_and_joints",
                                      ObjectAndJoints)
    msg = ObjectAndJoints()
    robot_state = RobotState()

    res = search_object(tf_listener)
    if res is None:
        rospy.logerr("No object found!")
        return

    poses, dims, names = res
    msg.object_poses =[p for p in poses]
    msg.box_dims = [d for d in dims]
    msg.object_names = names

    l_gripper_pos = PoseStamped()
    l_gripper_pos.header.frame_id = "l_wrist_roll_link"
    l_gripper_pos.pose.position.x = 0
    l_gripper_pos.pose.position.y = 0
    l_gripper_pos.pose.position.z = 0
    l_gripper_pos.pose.orientation.x = 0
    l_gripper_pos.pose.orientation.y = 0
    l_gripper_pos.pose.orientation.z = 0
    l_gripper_pos.pose.orientation.w = 0

    r_gripper_pos = PoseStamped()
    r_gripper_pos.header.frame_id = "r_wrist_roll_link"
    r_gripper_pos.pose.position.x = 0
    r_gripper_pos.pose.position.y = 0
    r_gripper_pos.pose.position.z = 0
    r_gripper_pos.pose.orientation.x = 0
    r_gripper_pos.pose.orientation.y = 0
    r_gripper_pos.pose.orientation.z = 0
    r_gripper_pos.pose.orientation.w = 0

    sleeper = rospy.Rate(20)

    raw_input("press a key to logging the trajectory...")

    while not rospy.is_shutdown():
        sleeper.sleep()
        if robot_state.last_joint_msg is None:
            continue

        msg.joints = robot_state.last_joint_msg
        msg.header.stamp = rospy.Time.now()

        l_gripper_pos.header.stamp = msg.joints.header.stamp
        r_gripper_pos.header.stamp = msg.joints.header.stamp
        try:
            tf_listener.waitForTransform("base_link",
                                         "r_wrist_roll_link",
                                         r_gripper_pos.header.stamp,
                                         rospy.Duration(1./10)
                                         )
            tf_listener.waitForTransform("base_link",
                                         "l_wrist_roll_link",
                                         l_gripper_pos.header.stamp,
                                         rospy.Duration(1./10)
                                         )
        except tf.Exception:
            rospy.logwarn("error while waiting for transform")
            continue
        try:
            msg.left_gripper_pose = tf_listener.transformPose("base_link", l_gripper_pos)
            msg.right_gripper_pose = tf_listener.transformPose("base_link", r_gripper_pos)
            obj_changes_pub.publish(msg)
        except tf.ExtrapolationException:
            rospy.logwarn("skipping extrapolation")
            continue




if __name__ == "__main__":

    rospy.init_node('log_objects_and_joints', anonymous=True)
    frame = "/base_link"
    tf_listener = tf.TransformListener()
    start_log(tf_listener)