#!/usr/bin/env python
"""
Copyright (c) 2012, Lorenzo Riano.
All rights reserved.
"""


import roslib; roslib.load_manifest("pr2_trajectory_markers")
import rospy
import pr2_control_utilities
from pr2_control_utilities.utils import (create_tuples_from_pose,
        normalize_trajectory)

from interactive_markers.interactive_marker_server import (
        InteractiveMarkerServer, InteractiveMarker,
        InteractiveMarkerControl,
        InteractiveMarkerFeedback
        )
from interactive_markers.menu_handler import MenuHandler

import utils
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty,EmptyResponse

class PR2TrajectoryMarkers(object):
    """
    A class to create and store a  trajectory for one PR2 arm. The created
    trajectory can be published as a PoseArray message.

    This class published on the following topics:
    ~trajectory_markers are the main interactive markers.
    ~trajectory_poses a markerarray to display the trajectory.
    ~trajectory_poses a posesarray with the resulting pose

    The class subscribes to the topic ~overwrite_trajectory_
    to change the stored trajectory. This is useful to resume working on a 
    trajectory after re-starting the node. The message type is PoseArray.

    A std_srvs/Empty service named ~execute_trajectory is provided to 
    externally trigger the execution of the trajectory.

    Constructor:
    TrajectoryMarkers(whicharm = "left")
    or
    TrajectoryMarkers(whicharm = "right")
    """
    def __init__(self, whicharm):
        self.whicharm = whicharm
        self.robot_state = pr2_control_utilities.RobotState()
        self.joint_controller = pr2_control_utilities.PR2JointMover(self.robot_state)
        self.planner = pr2_control_utilities.PR2MoveArm(self.joint_controller)
        self.server = InteractiveMarkerServer("~trajectory_markers")
        self.tf_listener = self.planner.tf_listener

        self.visualizer_pub = rospy.Publisher("~trajectory_markers_path",
                MarkerArray)
        self.trajectory_pub = rospy.Publisher("~trajectory_poses", 
                PoseArray)
        rospy.Subscriber("~overwrite_trajectory", 
                PoseArray,
                self.overwrite_trajectory)
        rospy.Service("~execute_trajectory", Empty, 
                self.execute_trajectory_srv)
        
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/base_link"
        int_marker.pose.position.x = 0.5
        int_marker.pose.position.z = 1.0
        int_marker.name = "move_" + whicharm + "_arm"
        int_marker.description = "Move the " + whicharm + " arm"
        int_marker.scale = 0.2
        self.server.insert(int_marker, self.main_callback)

        # create the main marker shape
        #color = (1,0,0,1)
        color = None
        self.gripper_marker = utils.makeGripperMarker(color=color)
        int_marker.controls.append(self.gripper_marker);
        #add the controls 
        utils.make_6DOF_marker(int_marker)

        self.int_marker = int_marker
        self.create_menu()
        self.server.applyChanges()

        self.trajectory = PoseArray()
        self.trajectory.header.frame_id = "/base_link"
        self.last_gripper_pose = None

        if whicharm == "right":
            self.ik_utils = self.planner.right_ik
        else:
            self.ik_utils = self.planner.left_ik

        rospy.loginfo("PR2TrajectoryMarkers (%s) is ready", whicharm)

    def create_menu(self):
        """
        Create and populates all the menu entries
        """
        menu_handler = MenuHandler()
        menu_handler.insert("Point the head", 
                callback = self.move_head)
        menu_handler.insert("Add position to trajectory", 
                callback = self.add_point)
        menu_handler.insert("Place marker over gripper", 
                callback = self.place_gripper)
        menu_handler.insert("Execute trjectory", 
                callback = self.execute_trajectory)
        menu_handler.insert("Clear trajectory", 
                callback = self.clear_trajectory)
        menu_handler.insert("Publish trajectory", 
                callback = self.publish_trajectory)
        menu_handler.insert("Move the arm (planning)", 
                callback = self.plan_arm)
        menu_handler.insert("Move the arm (collision-free)", 
                callback = self.collision_free_arm)
        menu_handler.insert("Move the arm to trajectory start (collision-free)",
                callback = self.arm_trajectory_start)
        menu_handler.insert("Update planning scene", 
                callback = self.update_planning)

        menu_handler.apply(self.server, self.int_marker.name)

    def main_callback(self, feedback):
        """
        If the mouse button is released change the gripper color according to 
        the IK result. Quite awkward, trying to get a nicer way to do it.
        """
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.last_gripper_pose =  feedback.pose
        #rospy.loginfo("Updating Marker: %d", feedback.event_type)
        #rospy.loginfo("POS: %s", feedback.pose.position)
        if (self.last_gripper_pose and 
                    feedback.event_type ==  InteractiveMarkerFeedback.MOUSE_UP):
            self.last_gripper_pose = None
            pos = PoseStamped()
            pos.header.frame_id = feedback.header.frame_id
            pos.pose = feedback.pose
            if self.whicharm == "right":
                ik = self.planner.check_ik_right_arm
            else:
                ik = self.planner.check_ik_left_arm
            
            if ik(pos):
                color = None
            else:
                color = (1,0,0,1)

            int_marker = self.server.get(self.int_marker.name)
            int_marker.controls.remove(self.gripper_marker)
            self.gripper_marker = utils.makeGripperMarker(color=color)
            int_marker.controls.append(self.gripper_marker)
            #rospy.loginfo("Control: %s", int_marker.controls)
            self.server.insert(int_marker, self.main_callback)
            self.server.setPose(int_marker.name, self.last_gripper_pose)
            self.server.applyChanges()

    def overwrite_trajectory(self, msg):
        self.trajectory = msg

    def add_point(self, feedback):
        """
        Add a point to self.trajectory if it is allowed by IK.
        """
        pos = PoseStamped()
        pos.header.frame_id = feedback.header.frame_id
        pos.pose = feedback.pose
        if self.whicharm == "right":
            ik = self.planner.check_ik_right_arm
        else:
            ik = self.planner.check_ik_left_arm
        
        if ik(pos):
            rospy.loginfo("Pose is reachable")
            self.trajectory.poses.append(feedback.pose)
        else:
            rospy.logerr("Pose is not reachable!")

    def place_gripper(self, feedback):
        """
        Move the marker where the gripper is
        """
        if self.whicharm == "right":
            gripper_pos = self.planner.get_right_gripper_pose()
        else:
            gripper_pos = self.planner.get_left_gripper_pose()
        self.server.setPose(self.int_marker.name, gripper_pos.pose, 
                gripper_pos.header)
        self.server.applyChanges()

    def execute_trajectory(self, feedback):
        """
        Executes the tracjectory memorized so far. It interpolates between
        the poses and creates suitable times and velocities.
        """

        traj = self.interpolate_poses()
        if len(traj) == 0:
            rospy.logerr("Something went wrong when interpolating")
            return

        times, vels = self.ik_utils.trajectory_times_and_vels(traj)
        if len(vels) == 0 or len(times) == 0:
            rospy.logerr("Something went wrong when finding the times")
            return 
        self.joint_controller.execute_trajectory(traj, times, vels,
                                                 self.whicharm,
                                                 wait = True)
        return 

    def execute_trajectory_srv(self, _):
        """Same as execute_trajectory, but as a service.
        """
        self.execute_trajectory(None)
        return EmptyResponse()

    def arm_trajectory_start(self, feedback):
        """
        Move the gripper to the first pose in the trajectory.
        """
        if len(self.trajectory.poses) == 0:
            rospy.logwarn("Empty trajectory!")
            return
        pose =  self.trajectory.poses[0]
        if self.whicharm == "right":
            moveit = self.planner.move_right_arm_non_collision
        else:
            moveit = self.planner.move_left_arm_non_collision
        pos, quat = create_tuples_from_pose(pose)
        res = moveit(pos, quat, self.trajectory.header.frame_id, 1.0)
        if not res:
            rospy.logerr("Something went wrong when moving")
            return

    def clear_trajectory(self, feedback):
        """
        Removes all the points stored so far
        """
        self.trajectory.poses = []

    def move_head(self, feedback):
        """
        Moves the head to face the marker
        """
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )

        print "Moving the head"
        self.joint_controller.time_to_reach = 1.0
        self.joint_controller.point_head_to(pos, frame)

    def plan_arm(self, feedback):
        """
        Moves the arm on the marker using motion collision-aware motion 
        planning.
        """
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )
        orientation = (feedback.pose.orientation.x,
               feedback.pose.orientation.y,
               feedback.pose.orientation.z,
               feedback.pose.orientation.w,
              )
      
        if self.whicharm == "right":
            rospy.loginfo("Moving the right arm")
            self.planner.move_right_arm(pos, orientation, frame, 2.0)
        else:
            rospy.loginfo("Moving the left arm")
            self.planner.move_left_arm(pos, orientation, frame, 2.0)

    def collision_free_arm(self, feedback):
        """
        Moves the rm on the marker using motion NON-collision-aware inverse
        kinematiks.
        """
        frame = feedback.header.frame_id
        pos = (feedback.pose.position.x,
               feedback.pose.position.y,
               feedback.pose.position.z,
              )
        orientation = (feedback.pose.orientation.x,
               feedback.pose.orientation.y,
               feedback.pose.orientation.z,
               feedback.pose.orientation.w,
              )
       
        if self.whicharm == "right":
            rospy.loginfo("Moving the right arm (non collision)")
            self.planner.move_right_arm_non_collision(pos, orientation, 
                                                      frame, 2.0)
        else:
            rospy.loginfo("Moving the left arm (non collision)")
            self.planner.move_left_arm_non_collision(pos, orientation, 
                                                     frame, 2.0)
    def update_planning(self, feedback):
        """
        Updates the planning scene.
        """
        self.planner.take_static_map()
        self.planner.update_planning_scene()

    def publish_trajectory(self, feedback):
        """
        Publishes the trajectory as a PoseArray message
        """
        self.trajectory_pub.publish(self.trajectory)

    def interpolate_poses(self):
        """
        Refines the trajectory by interpolating between the joints.
        """
        if len(self.trajectory.poses) < 2:
            rospy.logerr("At least two points in the trajectory are needed")
            return []

        if self.whicharm == "right":
            starting_angles = self.robot_state.right_arm_positions
        else:
            starting_angles = self.robot_state.left_arm_positions

        all_trajectory = [starting_angles]
        for i in xrange(len(self.trajectory.poses) - 1):
            start = PoseStamped()
            start.header = self.trajectory.header
            start.pose = self.trajectory.poses[i]
            
            end = PoseStamped()
            end.header = self.trajectory.header
            end.pose = self.trajectory.poses[i+1]
            
            traj, errs = self.ik_utils.check_cartesian_path(start, end,
                    all_trajectory[-1],
                    #starting_angles,
                    #pos_spacing = 0.05,
                    collision_aware = 0,
                    num_steps = 5,
                    )
            if any(e == 3 for e in errs):
                rospy.logerr("Error while running IK, codes are: %s", errs)
                return []

            filtered_traj = [t for (t,e) in zip(traj,errs) if e == 0]
            all_trajectory.extend(filtered_traj)

        all_trajectory = normalize_trajectory(all_trajectory, starting_angles)
        rospy.loginfo("New interpolated a trajectory with %d elements", 
                len(all_trajectory))
         
        return all_trajectory

    def publish_trajectory_markers(self, duration):
        """
        Publishes markers to visualize the current trajectory.

        Paremeters:
        duration: how long should the markers visualization last. If this
        function is called from a loop they last at least the loop rate.
        """
        if len(self.trajectory.poses) == 0:
            return
        msg = MarkerArray()
        marker_id = 0
        
        #creating the path connecting the axes
        path = Marker()
        path.header.frame_id = self.trajectory.header.frame_id
        path.ns = "path"
        path.action = Marker.ADD
        path.type = Marker.LINE_STRIP
        path.lifetime = rospy.Duration(duration)
        path.color.r = 1
        path.color.g = 0
        path.color.b = 1
        path.color.a = 1
        path.scale.x = 0.01
        path.id = marker_id

        marker_id += 1
        for pose in self.trajectory.poses:
            pos = PoseStamped()
            pos.header.frame_id = self.trajectory.header.frame_id
            pos.pose = pose
            
            markers = utils.axis_marker(pos, marker_id, "axes")
            msg.markers.extend(markers)

            path.points.append(pose.position)

            marker_id += 3 #3 axes 
        
        msg.markers.append(path)
        self.visualizer_pub.publish(msg)


if __name__ == "__main__":
    import sys    
    if len(sys.argv) < 2:
        rospy.logerr("Usage: %s [left|right]", sys.argv[0])
        sys.exit()
    if sys.argv[1] not in ("left", "right"):
        rospy.logerr("Usage: %s [left|right]", sys.argv[0])
        sys.exit()
    rospy.init_node("pr2_trajectory_markers_" + sys.argv[1])
    
    t = rospy.Rate(5)
    server =  PR2TrajectoryMarkers(sys.argv[1]) 
    while not rospy.is_shutdown():
        server.publish_trajectory_markers(1./5)
        t.sleep()
