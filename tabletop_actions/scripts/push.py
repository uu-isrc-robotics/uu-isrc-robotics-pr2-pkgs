#! /usr/bin/python
from codecs import ignore_errors
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)

import rospy
import tabletop_actions.pushers as pushers
import tabletop_actions.object_detector as object_detector 
import pr2_control_utilities

import numpy
import sys

def getout(msg):
    rospy.logerr(msg)
#    rospy.signal_shutdown(msg)
#    rospy.sleep(0.5)
#    sys.exit()
    return False

def find_primary_axis(box):
    box_dims = (box.box_dims.x,
                box.box_dims.y,
                box.box_dims.z)
    
    i = numpy.argmax(box_dims)
    axis = [0,0,0]
    axis[i] = 1
    return axis    

def push(robot_state, mover, pusher_l, pusher_r, detector):
    
    mover.time_to_reach = 1
    if not detector.point_head_at(mover,use_random = True):
        return getout("No object detected by the wide stereo")
    box =  detector.detect_bounding_box(use_random = True)
    if box is None:
        return getout("No box found for pre-pushing!")
    primary_axis_prepush = find_primary_axis(box)
    prev_box_position = (box.pose.pose.position.x,
                         box.pose.pose.position.y,
                         box.pose.pose.position.z)
    rospy.loginfo("Box pose before pushing: %s" %str(prev_box_position))
    rospy.loginfo("Primary axis before pushing: %s"%str(primary_axis_prepush))
    
    if box.pose.pose.position.y > 0:
        rospy.loginfo("Using the left pusher")
        pusher = pusher_l
    else:
        rospy.loginfo("Using the right pusher")
        pusher = pusher_r        
    
    if not pusher.push_object(box, ignore_errors=False):
        return getout("Something when wrong when pushing")
        
    
    #Pushing done, locating the object again
#    if not detector.point_head_at(mover, use_random = True):
#        return getout("No object detected by the wide stereo")
#    box =  detector.detect_bounding_box(use_random = True)
#    primary_axis_prepush = find_primary_axis(box)
#    rospy.loginfo("Primary axis after pushing: %s"%str(primary_axis_prepush))
#    
#    next_box_position = (box.pose.pose.position.x,
#                         box.pose.pose.position.y,
#                         box.pose.pose.position.z)
#    dist = numpy.sqrt( (next_box_position[0] - prev_box_position[0])**2 +
#                      (next_box_position[1] - prev_box_position[1])**2 +
#                      (next_box_position[2] - prev_box_position[2])**2 )
#    
#    rospy.loginfo("Distance travelled: %f"%dist)
#    rospy.loginfo("Done")

def main1():

    robot_state = pr2_control_utilities.RobotState()
    mover = pr2_control_utilities.PR2JointMover(robot_state)
    planner = pr2_control_utilities.PR2MoveArm(mover)
    detector = object_detector.ObjectDetector() 
    
    pusher_l = pushers.LeftArmCircularPusher(planner, robot_state)
    pusher_r = pushers.RightArmCircularPusher(planner, robot_state)
    
    push(robot_state, mover, pusher_l, pusher_r, detector)
    
def main2():
    
    robot_state = pr2_control_utilities.RobotState()
    mover = pr2_control_utilities.PR2JointMover(robot_state)
    planner = pr2_control_utilities.PR2MoveArm(mover)
    detector = object_detector.ObjectDetector() 
    
    pusher_l = pushers.LeftArmCircularPusher(planner, robot_state)
    pusher_r = pushers.RightArmCircularPusher(planner, robot_state)
    
    while not rospy.is_shutdown():
        push(robot_state, mover, pusher_l, pusher_r, detector)    

if __name__ == "__main__":
    rospy.init_node(PKG, anonymous=True)
    main1()
    