#! /usr/bin/python
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)

import rospy
from object_manipulation_msgs.msg import PickupAction, PickupGoal, PickupResult
from object_manipulation_msgs.msg import PlaceAction, PlaceGoal, PlaceResult
from object_manipulation_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped, PointStamped
import actionlib
from tf import transformations

from pr2_control_utilities import PR2MoveArm

PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup"
PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place"

class Grabber(object):
    def __init__(self):
        self.pickup_client = actionlib.SimpleActionClient(PICKUP_ACTION_NAME,
                                                          PickupAction)
        while not self.pickup_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("Witing for action client %s", PICKUP_ACTION_NAME)

        self.place_client = actionlib.SimpleActionClient(PLACE_ACTION_NAME,
                                                          PlaceAction)
        while not self.place_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("Witing for action client %s", PICKUP_ACTION_NAME)

        rospy.loginfo("Grabber ready")
        self.grasped_objects = {}
        self.last_pickup_result = None

    def pickup_object(self, graspable,
                      graspable_name,
                      table_name, which_arm,
                      execute = True,
                      desired_approach_distance = 0.2,
                      min_approach_distance = 0.05,
                      lift_desired_distance = 0.1,
                      lift_min_distance = 0.05,
                      allow_support_collision = True,
                      ignore_collisions = False,
                      wait = rospy.Duration()):
        """Picks up a previously detected object.

        Parameters:
        graspable: an object_manipulation_msgs/GraspableObject msg instance.
         This usually comes from a Detector.call_collision_map_processing call.
        graspable_name: the name of the object to graps. It is provided by
         Detector.call_collision_map_processing.
        table_name: the name of the table. Again provided by Detector.call_collision_map_processing.
        which_arm: left_arm or right_arm
        exectute: True|False, whether to execute the grasp or just check for 
          feasibility
        desired_approach_distance: how far the pre-grasp should ideally
         be away from the grasp
        min_approach_distance: how much distance between pre-grasp
         and grasp must actually be feasible for the grasp not to be rejected
        lift_desired_distance: the desired lift distance
        lift_min_distance: the min distance that must be considered
         feasible before the grasp is even attempted
        allow_support_collision: whether collisions between the gripper and the support
         surface should be acceptable during move from pre-grasp to grasp and
         during lift. Collisions when moving to the pre-grasp location are still not allowed even
         if this is set to true.
        ignore_collisions: set this to true if you want to ignore all collisions throughout
         the pickup and also move directly to the pre-grasp using
         Cartesian controllers
        wait: how long to wait for the action to be successful

        Return:
        a object_manipulation_msgs.PickupResult msg
        """

        rospy.loginfo("Calling the pickup action")
        pickup_goal = PickupGoal()
        pickup_goal.target = graspable
        pickup_goal.collision_object_name = graspable_name
        pickup_goal.collision_support_surface_name = table_name
        pickup_goal.only_perform_feasibility_test = not execute

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
        pickup_goal.allow_gripper_support_collision = allow_support_collision
        pickup_goal.ignore_collisions = ignore_collisions

        self.pickup_client.send_goal_and_wait(pickup_goal, wait)
        res = self.pickup_client.get_result()
        assert isinstance(res, PickupResult)
        manipulation_result = res.manipulation_result

        if manipulation_result.value != manipulation_result.SUCCESS:
            rospy.logerr("Error during pickup")
            return None

        self.last_pickup_result = res
        return res

    def place_object(self, place_locations,
                     grasp,
                     arm_name,
                     graspable_name,
                     table_name,
                     frame_id = "/base_link",
                     place_padding = 0.02,
                     desired_retreat_distance = 0.1,
                     min_retreat_distance = 0.05,
                     desired_distance = 0.1,
                     min_distance = 0.05,
                     wait = rospy.Duration()
                     ):
        """
        Parameters:

        locations: list of (x,y,z) coordinates
        grapsp: a object_manipulation_msgs/Grasp instance
        """
        rospy.loginfo("Calling the place action")

        assert isinstance(grasp, Grasp)

        gripper_transl = (grasp.grasp_pose.position.x,
                          grasp.grasp_pose.position.y,
                          grasp.grasp_pose.position.z)

        place_poses = []
        for place in place_locations:
            place_pose = PoseStamped()
            place_pose.header.frame_id = frame_id
            #no change in orientation
            place_pose.pose.orientation.w = 1

            newpos = (place[0] - gripper_transl[0],
                      place[1] - gripper_transl[1],
                      place[2] - gripper_transl[2],
                      )
            place_pose.pose.position.x = newpos[0]
            place_pose.pose.position.y = newpos[1]
            place_pose.pose.position.z = newpos[2]
            place_poses.append(place_pose)

        place_goal = PlaceGoal()

        place_goal.place_locations = place_poses

        place_goal.collision_object_name = graspable_name
        place_goal.collision_support_surface_name = table_name

        place_goal.grasp = grasp
        place_goal.arm_name = arm_name

        place_goal.place_padding = place_padding
        place_goal.desired_retreat_distance = desired_retreat_distance
        place_goal.min_retreat_distance = min_retreat_distance

        place_goal.approach.direction.header.frame_id = "base_link"
        place_goal.approach.direction.vector.x = 0
        place_goal.approach.direction.vector.y = 0
        place_goal.approach.direction.vector.z = -1

        place_goal.approach.desired_distance = desired_distance
        place_goal.approach.min_distance = min_distance
        place_goal.use_reactive_place = False

        self.place_client.send_goal_and_wait(place_goal, wait)
        res = self.place_client.get_result()
        assert isinstance(res, PlaceResult)
        manipulation_result = res.manipulation_result

        if manipulation_result.value != manipulation_result.SUCCESS:
            rospy.logerr("Error during place")
            return None

        return res


    def place_reactive(self, 
                       arm_mover,
                       place_locations,
                       arm_name,
                       object_height,
                       frame_id = "/base_link",
                       check_collisition = True,
                       ):
        """Very experimental reactive placing. It tries a set of locations listed
        as (x,y,z) in place_locations, and for each of them it tries to move the arm
        in place using the PR2MoveArm arm_mover.
        The final position will have the gripper pointing downward. The object's
        height is taken into consideration.
        """
        
        assert isinstance(arm_mover, PR2MoveArm)
        listener = arm_mover.tf_listener
        if type(arm_name) is not str:
            rospy.logerr("Error, arm_name is wrong type: %s", arm_name)
            return False
        
        if arm_name.startswith("right"):
            if check_collisition:
                move = arm_mover.move_right_arm
            else:
                move = arm_mover.move_right_arm_non_collision
            open_gripper = arm_mover.joint_mover.open_right_gripper
        else:
            if check_collisition:
                move = arm_mover.move_left_arm
            else:
                move = arm_mover.move_left_arm_non_collision
            open_gripper = arm_mover.joint_mover.open_left_gripper

        listener.waitForTransform("/base_link", frame_id, rospy.Time(), rospy.Duration(10))
        
        for position in place_locations:            
            
            orientation = (0,0.7,0,0.7)
            point = PointStamped()
            point.header.frame_id = frame_id
            point.point.x = position[0]
            point.point.y = position[1]
            point.point.z = position[2] + object_height + 0.18
            newpoint = listener.transformPoint("/base_link", point)
                        
            newposition = (newpoint.point.x,
                           newpoint.point.y,
                           newpoint.point.z)
            
            rospy.loginfo("Trying position %s, frame: %s", newposition, newpoint.header.frame_id)
            if move(newposition, orientation, newpoint.header.frame_id, 10):
                rospy.loginfo("PLace %s was good! Now opening the gripper",
                              position)
                open_gripper()
                return True
        rospy.logwarn("Could not place object into desired position")
        return False
            
            
            
            
        