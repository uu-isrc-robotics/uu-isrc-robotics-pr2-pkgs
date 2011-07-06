#! /usr/bin/python
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)

import rospy
import tabletop_actions.object_detector as object_detector
import tabletop_actions.object_pickup as object_pickup
import pr2_control_utilities

import sys
import random
from tf import transformations
import math
from visualization_msgs.msg import Marker

def getout(msg):
    rospy.logerr(msg)
    sys.exit()    

def draw_pose(pose, orientation, frame):
    
    collision_objects__markers_pub = rospy.Publisher("release_pose", 
                                                      Marker, 
                                                      latch=True)
    
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "gripper_pointing"
    marker.type = Marker.ARROW
    
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]

    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    
    marker.id = 0
    marker.action = Marker.ADD
    marker.header.frame_id = frame
    
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    
    marker.color.a = 1.0
    marker.color.r = 1.0
    
    collision_objects__markers_pub.publish(marker)

def main():
    robot_state = pr2_control_utilities.RobotState()
    joint_mover = pr2_control_utilities.PR2JointMover(robot_state)    
    detector = object_detector.ObjectDetector()
    grabber = object_pickup.Grabber()
    planner = pr2_control_utilities.PR2MoveArm(joint_mover)
    base_mover = pr2_control_utilities.PR2BaseMover(planner.tf_listener)
    
    detection_msg = detector.detect_wide()
    if detection_msg is None:
        getout("No objects found!")
    
    nclusters = len(detection_msg.detection.clusters)
    if nclusters == 1:
        getout("only one object found!")    
        
    else:
        rospy.loginfo("Detected %d objects" % 
                      nclusters)
    collision_proc = detector.call_collision_map_processing(detection_msg)
    
    cluster1 = detection_msg.detection.clusters[0]
    cluster2 = detection_msg.detection.clusters[1]
    box1 = detector.detect_bounding_box(cluster1)
    box2 = detector.detect_bounding_box(cluster2)
    
    #take the left box
    if box1.pose.pose.position.y <  box2.pose.pose.position.y:
        box_to_pickup = box1
        i1 = 0
        box_to_stack = box2
        i2 = 1
    else:
        box_to_pickup = box2
        i1 = 1
        box_to_stack = box1
        i2 = 0
    
    rospy.loginfo("Picking up object %d to be put over object %d" %(i1, i2))
    
    detector.draw_bounding_box(0, box_to_pickup)    
    detector.draw_bounding_box(1, box_to_stack)
    
    if box_to_pickup.pose.pose.position.y > 0:
        rospy.loginfo("Using the left arm")
        whicharm = "left_arm"
    else:
        rospy.loginfo("Using the right arm")
        whicharm = "right_arm"
    
    grabber.pickup_object(collision_proc, whicharm, index = i1,
                          min_approach_distance=0.05,
                          lift_desired_distance=0.4)
    
    euler_angles = (0., math.pi/2., math.pi/2.)
    desired_orientation = transformations.quaternion_from_euler(euler_angles[0],
                                                   euler_angles[1],
                                                   euler_angles[2])
    
    base_mover.drive_to_displacement((0, box_to_stack.pose.pose.position.y, 0))
    box_to_stack.pose.pose.position.y = 0 #after displacement
    box_pose = (box_to_stack.pose.pose.position.x,
                box_to_stack.pose.pose.position.y,
                box_to_stack.pose.pose.position.z)
    
    
    gripper_length = 0.15
    offset_z = box_to_stack.box_dims.y/2. + gripper_length + 0.1
    desired_pose = (box_pose[0],
                    box_pose[1],
                    box_pose[2] + offset_z)
    
    draw_pose(desired_pose, desired_orientation, "base_link")
    
    if whicharm == "right_arm":
        planner.move_right_arm_non_collision(desired_pose, 
                               desired_orientation,
                               "base_link",
                               time_required = 8.0)
        joint_mover.open_right_gripper(True)
    else:
        planner.move_left_arm_non_collision(desired_pose, 
                               desired_orientation,
                               "base_link",
                               time_required = 8.0)
        joint_mover.open_right_gripper(True)
    
    rospy.sleep(1.0)
    
if __name__ == "__main__":
    rospy.init_node(PKG, anonymous=True)
    main()