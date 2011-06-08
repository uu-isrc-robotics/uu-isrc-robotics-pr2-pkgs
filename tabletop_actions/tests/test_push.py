#! /usr/bin/python
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)

import rospy
import tabletop_actions.utils as utils

import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from mapping_msgs.msg import CollisionObject
from planning_environment_msgs.srv import GetCollisionObjects, GetCollisionObjectsRequest
from tf import transformations

import tabletop_actions.object_detector as object_detector 
import pr2_control_utilities
import numpy
import math
import cPickle

def draw_arrow(start_push, end_push, frame):
    pushing_objects_pub = rospy.Publisher("pushing_markers", Marker, latch=True)
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    
    marker.header.frame_id = frame
    marker.id = 0
    marker.ns = "pushing_arrow"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    
    marker.points.append(Point(start_push[0],
                               start_push[1],
                               start_push[2]))
    marker.points.append(Point(end_push[0],
                               end_push[1],
                               end_push[2]))
    
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    marker.lifetime = rospy.Duration()
    pushing_objects_pub.publish(marker) 

def draw_object(object):
    rospy.wait_for_service("/environment_server/get_collision_objects")
    object_finder = rospy.ServiceProxy("/environment_server/get_collision_objects", GetCollisionObjects)
    
    req = GetCollisionObjectsRequest(False)
    res = object_finder.call(req)
    
    found_obj = None
    for o in res.collision_objects:
        if o.id == object:
            found_obj = o
            break
    if found_obj is None:
        rospy.logwarn("No object with id %s found"%object)
        return
    
#    rospy.loginfo("Found the the object \n%s"%str(found_obj))
        
    collision_objects__markers_pub = rospy.Publisher("collision_objects__markers", Marker, latch=True)
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "contact_region"
    marker.type = Marker.CUBE
    marker.id = 100
    marker.action = Marker.ADD
    marker.header.frame_id = found_obj.header.frame_id
    
    marker.scale.x = found_obj.shapes[0].dimensions[0]
    marker.scale.y = found_obj.shapes[0].dimensions[1]
    marker.scale.z = found_obj.shapes[0].dimensions[2]
    
    marker.pose = found_obj.poses[0] 
    
    marker.color.a = 0.8
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.0
    
    marker.lifetime = rospy.Duration()
    
    collision_objects__markers_pub.publish(marker)
    
def draw_poses(poses, frame):
    rospy.loginfo("Drawing the poses")
    collision_objects__markers_pub = rospy.Publisher("circle_around__markers", Marker, latch=True)
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "circle_around"
    marker.type = Marker.POINTS
    marker.pose.orientation.w = 1.0
    marker.id = 1000
    marker.action = Marker.ADD
    marker.header.frame_id = frame
    
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    
    marker.color.b = 1.0
    marker.color.a = 1.0
    
    for p in poses:
        marker.points.append(Point( p[0], p[1],p[2] ))
    
    collision_objects__markers_pub.publish(marker)

def get_circular_pushing_poses(box_msg, num_points, z_offset = 0):
    boxpose = (box_msg.pose.pose.position.x,
               box_msg.pose.pose.position.y,
               box_msg.pose.pose.position.z)
    boxangle = math.atan2(boxpose[1], boxpose[0])
    boxdist = math.sqrt(boxpose[0]*boxpose[0] + boxpose[1]*boxpose[1])
    
    angle_start = boxangle - math.pi/8.0
    angle_end = boxangle + math.pi/8.0
    poses = []
    all_angles = numpy.linspace(angle_start, angle_end, num_points, endpoint=True)
    for alpha in all_angles:
        poses.append((boxdist * math.cos(alpha),
                      boxdist * math.sin(alpha),
                      boxpose[2] + z_offset))
    return poses

def get_linear_pushing_poses(box_msg, z_offset = 0):
    boxpose = (box_msg.pose.pose.position.x,
               box_msg.pose.pose.position.y,
               box_msg.pose.pose.position.z)
    
    start_push = (boxpose[0],
                  boxpose[1] -0.25,
                  boxpose[2] + z_offset)
    end_push = (boxpose[0],
                  boxpose[1] + 0.25,
                  boxpose[2] + z_offset)
    
    return start_push, end_push
    

def frontal_push_poses(boxpose):
    start_offset =  -0.28
    end_offset = 0.0
    z_offset = 0.0
    start_push = (boxpose[0] + start_offset,
                  boxpose[1],
                  boxpose[2] + z_offset)
    end_push = (boxpose[0] + end_offset,
                  boxpose[1],
                  boxpose[2] + z_offset)
    
    return [start_push, end_push]


def gripper_front_angles():
    
    euler_angles = (0., math.pi/2., 0)
    start_angles = transformations.quaternion_from_euler(euler_angles[0],
                                                            euler_angles[1],
                                                            euler_angles[2])
    end_angles = start_angles
    return start_angles, end_angles
    
def gripper_down_angles():
    euler_angles = (0., math.pi/2., 0)
    start_angles = transformations.quaternion_from_euler(euler_angles[0],
                                                            euler_angles[1],
                                                            euler_angles[2])
    end_angles = start_angles
    return start_angles, end_angles

def gripper_down_horizontal_angles():
    euler_angles = (0., math.pi/2., math.pi/2.)
    start_angles = transformations.quaternion_from_euler(euler_angles[0],
                                                            euler_angles[1],
                                                            euler_angles[2])
    end_angles = start_angles
    return start_angles, end_angles

def find_primary_axis(box):
    from itertools import izip, count
    # given an iterable of pairs return the key corresponding to the greatest value
    def argmax(pairs):
        return max(pairs, key=lambda x: x[1])[0]
    
    # given an iterable of values return the index of the greatest value
    def argmax_index(values):
        return argmax(izip(count(), values))

    
    box_dims = (box.box_dims.x,
                box.box_dims.y,
                box.box_dims.z)
    
    i = argmax_index(box_dims)
    axis = [0,0,0]
    axis[i] = 1
    return axis    
    

def getout(msg):
    rospy.logerr(msg)
    rospy.signal_shutdown(msg)
    rospy.sleep(0.5)
    sys.exit()
    

def main():
    rospy.init_node(PKG, anonymous=True)
    
    ik = pr2_control_utilities.IKUtilities("right")
    robot_state = pr2_control_utilities.RobotState()    
    mover = pr2_control_utilities.PR2JointMover(robot_state)    
    planner = pr2_control_utilities.PR2MoveArm(ik, mover)
    detector = object_detector.ObjectDetector() 
    
    collision_objects_pub = rospy.Publisher("collision_object", CollisionObject)
    initial_head = robot_state.head_positions[:]
    mover.time_to_reach = 1
    
    if not detector.point_head_at(mover):
        getout("No object detected by the wide stereo")
    box =  detector.detect_bounding_box()
    if box is None:
        getout("No box found for pre-pushing!")
    primary_axis_prepush = find_primary_axis(box)
    prev_box_position = (box.pose.pose.position.x,
                         box.pose.pose.position.y,
                         box.pose.pose.position.z)
    rospy.loginfo("Box pose before pushing: %s" %str(prev_box_position))
    rospy.loginfo("Primary axis before pushing: %s"%str(primary_axis_prepush))
    
    #adding the object
    object_id = "graspable_bottle"
    collision_object_msg = utils.build_object_from_box(box, object_id)
    collision_objects_pub.publish(collision_object_msg)    
    
    draw_object(object_id)
    frame = box.pose.header.frame_id
    
    traj_poses = get_circular_pushing_poses(box, 10, z_offset=0.2)
    draw_poses(traj_poses, frame)    
    
    starting_angles, end_angles = gripper_down_horizontal_angles()

    collision_operation = utils.build_collision_operations("right_arm", object_id)    
    allowed_contact_specification = utils.build_allowed_contact_specification(box) 
    
    previous_pose = pr2_control_utilities.PR2JointMover(robot_state)
    previous_pose.store_targets()

    #moving to pre_push position
    mover.close_right_gripper(True)
    start_push = traj_poses[0]
    if planner.move_right_arm(start_push, starting_angles, frame, waiting_time = 30, 
                              ordered_collision_operations = collision_operation):
        rospy.loginfo("Planning ok")
    else:
        getout("Planning not done")
        
    whole_angles = [starting_angles] * len(traj_poses)
    planner.joint_mover.time_to_reach = 5.0
    mover.set_head_state(initial_head)
    if planner.move_right_arm_trajectory_non_collision(traj_poses, 
                                                       whole_angles, 
                                                       frame, 
                                                       max_vel = 0.4,
                                                       ignore_errors=False,
                                                       ):
        rospy.loginfo("Pushing ok")
    else:
        getout("Pushing not done")
    
    rospy.loginfo("Moving back to the original place")
    previous_pose.time_to_reach = 10
    previous_pose.execute_and_wait()
    
    head_aim = (traj_poses[-1][0],
                traj_poses[-1][1],
                traj_poses[-1][2]-0.4)
    
    previous_pose.point_head_to(head_aim, frame)
    
    if not detector.point_head_at(mover):
        getout("No object detected by the wide stereo")
    box =  detector.detect_bounding_box()
    primary_axis_prepush = find_primary_axis(box)
    rospy.loginfo("Primary axis after pushing: %s"%str(primary_axis_prepush))
    
    next_box_position = (box.pose.pose.position.x,
                         box.pose.pose.position.y,
                         box.pose.pose.position.z)
    dist = math.sqrt( (next_box_position[0] - prev_box_position[0])**2 +
                      (next_box_position[1] - prev_box_position[1])**2 +
                      (next_box_position[2] - prev_box_position[2])**2 )
    
    rospy.loginfo("Distance travelled: %f"%dist)
    rospy.loginfo("Done")
    rospy.spin()
    
if __name__ == "__main__":
    main()
    

