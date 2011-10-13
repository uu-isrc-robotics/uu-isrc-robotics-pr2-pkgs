#! /usr/bin/python
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)

import rospy
from object_manipulation_msgs.msg import PickupAction, PickupGoal
from object_manipulation_msgs.msg import PlaceAction
import actionlib

PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup"

class Grabber(object):
    def __init__(self):
        self.pickup_client = actionlib.SimpleActionClient(PICKUP_ACTION_NAME, 
                                                          PickupAction)
        while not self.pickup_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("Witing for action client %s", PICKUP_ACTION_NAME)
        
        rospy.loginfo("Grabber ready")
          
    def pickup_object(self, collision_map_processing_msg, which_arm,
                      index = 0, desired_approach_distance = 0.2,
                      min_approach_distance = 0.05,
                      lift_desired_distance = 0.1,
                      lift_min_distance = 0.05,
                      wait = rospy.Duration()):
        '''
        Picks up a previously detected object
        @param collision_map_processing_msg: The message resulting from calling
        a TabletopCollisionMapProcessing service.
        @param index: The index of the object to grasp, if more than one 
        @param which_arm: left_arm or right_arm 
        @param desired_approach_distance: how far the pre-grasp should ideally
        be away from the grasp 
        @param min_approach_distance: how much distance between pre-grasp 
        and grasp must actually be feasible for the grasp not to be rejected
        @param lift_desired_distance: the desired lift distance
        @param lift_min_distance: the min distance that must be considered 
        feasible before the grasp is even attempted
        @param wait: how long to wait for the action to be successful 
        '''
        
        pickup_goal = PickupGoal()
        pickup_goal.target = collision_map_processing_msg.graspable_objects[index]
        pickup_goal.collision_object_name = collision_map_processing_msg.collision_object_names[index]
        pickup_goal.collision_support_surface_name = collision_map_processing_msg.collision_support_surface_name
        
        pickup_goal.arm_name = which_arm
        
        pickup_goal.lift.direction.header.frame_id = "base_link"
        pickup_goal.lift.direction.vector.x = 0
        pickup_goal.lift.direction.vector.y = 0
        pickup_goal.lift.direction.vector.z = 1
        
        pickup_goal.lift.desired_distance = lift_desired_distance
        pickup_goal.lift.min_distance = lift_min_distance
    #    do not use tactile-based grasping or tactile-based lift
        pickup_goal.use_reactive_lift = False
        pickup_goal.use_reactive_execution = False
        
        self.pickup_client.send_goal_and_wait(pickup_goal, wait)
        