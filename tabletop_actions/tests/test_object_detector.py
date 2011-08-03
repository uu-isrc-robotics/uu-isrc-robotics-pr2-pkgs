#! /usr/bin/python

PKG="tabletop_explorer"

import roslib
roslib.load_manifest(PKG)

import rospy
import pr2_control_utilities
import tabletop_actions.object_detector as object_detector
import tf 
from tf import transformations

import sys

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
    
    
    table = detector.last_detection_msg.detection.table
    table_quat = (table.pose.pose.orientation.x,
                  table.pose.pose.orientation.y,
                  table.pose.pose.orientation.z,
                  table.pose.pose.orientation.w)
    table_angles = transformations.euler_from_quaternion(table_quat)
    
    rospy.loginfo("Table pos: %s", str(table.pose))    
    rospy.loginfo("Table angle: %s", str(table_angles))

    box = detector.last_box_msg
    box_pose = box.pose
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform(box_pose.header.frame_id,
                                         "/odom_combined",
                                         rospy.Time(), rospy.Duration(4.0))
    box_pos_odom = tf_listener.transformPose("/odom_combined", box_pose)
    rospy.loginfo("Pos in odom: \n%s", str(box_pos_odom))
    joint_mover.point_head_to((box_pos_odom.pose.position.x,
                              box_pos_odom.pose.position.y,
                              box_pos_odom.pose.position.z),
                              box_pose.header.frame_id)
    
