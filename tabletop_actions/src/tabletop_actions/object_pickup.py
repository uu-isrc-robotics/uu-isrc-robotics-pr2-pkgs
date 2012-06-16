#! /usr/bin/python
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)

import rospy
from object_manipulation_msgs.msg import PickupAction, PickupGoal, PickupResult
from object_manipulation_msgs.msg import PlaceAction, PlaceGoal, PlaceResult
from object_manipulation_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped
import actionlib
from tf import transformations

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

        place_goal.approach.desired_distance = desired_distance;
        place_goal.approach.min_distance = min_distance;
        place_goal.use_reactive_place = False;

        self.place_client.send_goal_and_wait(place_goal, wait)
        res = self.place_client.get_result()
        assert isinstance(res, PlaceResult)
        manipulation_result = res.manipulation_result

        if manipulation_result.value != manipulation_result.SUCCESS:
            rospy.logerr("Error during place")
            return None

        return res

