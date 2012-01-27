import rospy
from visualization_msgs.msg import Marker
from arm_navigation_msgs.msg import CollisionObject
from geometry_msgs.msg import Point
from arm_navigation_msgs.msg import Shape
from tf import transformations

import pr2_control_utilities
import exceptions
import math
import numpy

class Pusher(object):
    def __init__(self, planner, robot_state, pre_pushing_pose = None):
        super(Pusher, self).__init__()
        self.planner = planner
        self.collision_objects__markers_pub = rospy.Publisher("pusher_trajectory", 
                                                              Marker, 
                                                              tcp_nodelay=True,
                                                              latch=True)
        self.collision_objects_pub = rospy.Publisher("collision_object", CollisionObject)
        self.robot_state = robot_state
        self.mover = pr2_control_utilities.PR2JointMover(robot_state)
        self.err_msg = ""

    def test_push(self, box_msg, ignore_errors=False):
        """
        Tests if the object specified in box_msg is pushable.
        Additionally draws the pushing trajectory by publishing markers.

        Parameters:
        box_msg: A FindClusterBoundingBox service response message. 
        ignore_errors: wether or not to ignore single errors in the IK
                    calculation. Sometimes single points in the trajectory can
                    be ignored, but doing so might not guarantee that the robot
                    will follow the correct trajectory.
        
        Return:
        (traj_poses, traj_angles): two lists with the trajectory poses and
                angles which can be provided to perform_push. Returns None
                if no trajectory was found.
        """
        self.planner.update_planning_scene()
        self.err_msg = ""
        if self.which_arm() == "right_arm":
            trajectory_creator = self.planner.create_right_arm_trjectory_non_collision
        elif self.which_arm() == "left_arm":
            trajectory_creator = self.planner.create_left_arm_trjectory_non_collision
        else:

            rospy.logerr("Non existing arm being used!")
            return None
        
        frame = box_msg.pose.header.frame_id
        
        traj_poses = self.get_pushing_poses(box_msg)
        traj_angles = self.get_pushing_angles(box_msg)
        self.__draw_poses(traj_poses, frame)
        
        
        rospy.loginfo("Testing if pushing trajectory is feasible")
        traj_angles = [traj_angles] * len(traj_poses)
        res = trajectory_creator(traj_poses,
                                 traj_angles,
                                 frame,
                                 max_vel=0.4,
                                 ignore_errors=ignore_errors,
                                 normalize=False)
        if res:
            rospy.loginfo("Pushing trajectory is ok")
            return (traj_poses, traj_angles)
        else:
            self.err_msg = "IK Error"
            rospy.logerr("The trajectory is not feasible")
            return None

    def perform_push(self, box_msg, traj_poses, traj_angles, 
                    max_vel = 0.2, ignore_errors = False,
                    normalize = True):
        """
        Pushes an object represented by a cluster bounding box. The trajectory
        has to be calculated beforhand by test_pusher. The arm will move back
        to the initial position after pushing.
        
        Parameters:
        box_msg: a FindClusterBoundingBox service response message. 
        traj_poses: the trajectory poses found by test_push.
        traj_angles: the trajectory angles found by test_push.
        max_vel: the maximum velocity to apply to the joints.
        ignore_errors: wether or not to ignore single errors in the IK
                    calculation. Sometimes single points in the trajectory can
                    be ignored, but doing so might not guarantee that the robot
                    will follow the correct trajectory.
        normalize: wether or not to normalize the trajectory so that the joints
                    will move less while following the same trajectory. It could
                    be computationally expensive.

        Returns: True on success, False otherwise.
        """
        
        frame = box_msg.pose.header.frame_id
        initial_head_position = self.robot_state.head_positions[:]
        if self.which_arm() == "right_arm":
            move_arm = self.planner.move_right_arm
            pre_push_joints = self.robot_state.right_arm_positions[:]
            trajectory_mover = self.planner.move_right_arm_trajectory_non_collision
        elif self.which_arm() == "left_arm":
            move_arm = self.planner.move_left_arm
            pre_push_joints = self.robot_state.left_arm_positions[:]
            trajectory_mover = self.planner.move_left_arm_trajectory_non_collision
        else:
            return self.__getout("Non existing arm being used!")

        #creating the allow_contacts_specification.. useless right now!
        object_id = "pushable_object"
        collision_object_msg = self.__build_object_from_box(box_msg, object_id)
        self.collision_objects_pub.publish(collision_object_msg)  
        
        collision_operation = self.planner.build_collision_operations(self.which_arm(), 
                                                                 object_id)
        dimensions = [box_msg.box_dims.x,
                      box_msg.box_dims.y,
                      box_msg.box_dims.z]    
        allowed_contact_specification = self.planner.build_allowed_contact_specification(box_msg.pose,
                                                                                         dimensions)
        allowed_contact_specification = [allowed_contact_specification]
        
        #moving to pre_push position
        self.mover.close_right_gripper(True)
        self.mover.close_left_gripper(True)
        start_push = traj_poses[0]
        start_angle = traj_angles[0]
        
        rospy.loginfo("Moving to pre-push")
        if move_arm(start_push, start_angle, frame, waiting_time = 30, 
                                  ordered_collision_operations = collision_operation,
                                  allowed_contacts = allowed_contact_specification):
            rospy.loginfo("Pre-push movement ok")
        else:
            self.err_msg = "Planning Error"
            return self.__getout("Pre-push planning returned with an error")
        
        
        self.planner.joint_mover.time_to_reach = 5.0
        self.mover.set_head_state(initial_head_position)
        rospy.loginfo("Starting the push")
        if not trajectory_mover(traj_poses,
                         traj_angles,
                         frame,
                         max_vel=max_vel,
                         ignore_errors=ignore_errors,
                         normalize=normalize):
            rospy.logerr("Strangely something when wrong when actually pushing!")
            rospy.loginfo("Moving the %s back" % self.which_arm())
            self.mover.set_arm_state(pre_push_joints,self.which_arm(),wait=True)
            self.err_msg = "IK Error"
            return False
            
        
#        self.mover.execute_trajectory(trajectory, times, vels, self.which_arm(), True)
         
        rospy.loginfo("Moving the %s back" % self.which_arm())
        self.mover.set_arm_state(pre_push_joints,self.which_arm(),wait=True)
        return True

  
    def push_object(self, box_msg, ignore_errors=False, normalize=True,
                    max_vel = 0.2):
        """
        Executes test_push and, if successfull, perform_push. Check the two
        methods for a description of parameters.
    
        Return:
        True on success, False otherwise.    
        """
        ret = self.test_push(box_msg, ignore_errors)
        if ret:
            traj_poses, traj_angles = ret
            return self.perform_push(box_msg, traj_poses, traj_angles,
                                     ignore_errors, normalize,
                                     max_vel)
        else:
            return False

    
    def __push_object(self, box_msg, ignore_errors = False, normalize = True,
                    max_vel = 0.2):

        
        self.planner.update_planning_scene()
        self.err_msg = ""
        if self.which_arm() == "right_arm":
            move_arm = self.planner.move_right_arm
            pre_push_joints = self.robot_state.right_arm_positions[:]
            trajectory_creator = self.planner.create_right_arm_trjectory_non_collision
            trajectory_mover = self.planner.move_right_arm_trajectory_non_collision
        elif self.which_arm() == "left_arm":
            move_arm = self.planner.move_left_arm
            pre_push_joints = self.robot_state.left_arm_positions[:]
            trajectory_creator = self.planner.create_left_arm_trjectory_non_collision
            trajectory_mover = self.planner.move_left_arm_trajectory_non_collision
        else:
            return self.__getout("Non existing arm being used!")
        
        initial_head_position = self.robot_state.head_positions[:]
        frame = box_msg.pose.header.frame_id
        
        traj_poses = self.get_pushing_poses(box_msg)
        traj_angles = self.get_pushing_angles(box_msg)
        self.__draw_poses(traj_poses, frame)
        
        
        rospy.loginfo("Testing is pushing trajectory is feasible")
        whole_angles = [traj_angles] * len(traj_poses)
        res = trajectory_creator(traj_poses,
                                 whole_angles,
                                 frame,
                                 max_vel=0.4,
                                 ignore_errors=ignore_errors,
                                 normalize=False)
        if res:
            rospy.loginfo("Pushing trajectory is ok")
#            trajectory, times, vels = res
        else:
            self.err_msg = "IK Error"
            return self.__getout("The trajectory is not feasible")
        
        
        #creating the allow_contacts_specification.. useless right now!
        object_id = "pushable_object"
        collision_object_msg = self.__build_object_from_box(box_msg, object_id)
        self.collision_objects_pub.publish(collision_object_msg)  
        
        collision_operation = self.planner.build_collision_operations(self.which_arm(), 
                                                                 object_id)
        dimensions = [box_msg.box_dims.x,
                      box_msg.box_dims.y,
                      box_msg.box_dims.z]    
        allowed_contact_specification = self.planner.build_allowed_contact_specification(box_msg.pose,
                                                                                         dimensions)
        allowed_contact_specification = [allowed_contact_specification]
        
        #moving to pre_push position
        self.mover.close_right_gripper(True)
        self.mover.close_left_gripper(True)
        start_push = traj_poses[0]
        
        rospy.loginfo("Moving to pre-push")
        if move_arm(start_push, traj_angles, frame, waiting_time = 30, 
                                  ordered_collision_operations = collision_operation,
                                  allowed_contacts = allowed_contact_specification):
            rospy.loginfo("Pre-push movement ok")
        else:
            self.err_msg = "Planning Error"
            return self.__getout("Pre-push planning returned with an error")
        
        
        self.planner.joint_mover.time_to_reach = 5.0
        self.mover.set_head_state(initial_head_position)
        rospy.loginfo("Starting the push")
        if not trajectory_mover(traj_poses,
                         whole_angles,
                         frame,
                         max_vel=max_vel,
                         ignore_errors=ignore_errors,
                         normalize=normalize):
            rospy.logerr("Strangely something when wrong when actually pushing!")
            rospy.loginfo("Moving the %s back" % self.which_arm())
            self.mover.set_arm_state(pre_push_joints,self.which_arm(),wait=True)
            self.err_msg = "IK Error"
            return False
            
        
#        self.mover.execute_trajectory(trajectory, times, vels, self.which_arm(), True)
         
        rospy.loginfo("Moving the %s back" % self.which_arm())
        self.mover.set_arm_state(pre_push_joints,self.which_arm(),wait=True)
        return True
    
    def __getout(self, msg):
        rospy.logerr(msg)
        return False
    
    def get_pushing_angles(self, box_msg):
        raise exceptions.NotImplementedError("Do not use a %s class directly!" % self.__class__)
    
    def get_pushing_poses(self, box_msg):
        raise exceptions.NotImplementedError("Do not use a %s class directly!" % self.__class__)    
    
    def which_arm(self):
        raise exceptions.NotImplementedError("Do not use a %s class directly!" % self.__class__)
    
    def __draw_poses(self, poses, frame):
        rospy.loginfo("Drawing the poses")
       
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
        
        self.collision_objects__markers_pub.publish(marker)
    
    def __build_object_from_box(self, box_msg, object_id):
        msg = CollisionObject()
        
        msg.header.frame_id = box_msg.pose.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.id = object_id
        msg.operation.operation = msg.operation.ADD
        
        shape = Shape()
        shape.type = shape.BOX
        scale = 1.0
        shape.dimensions = [scale*box_msg.box_dims.x,
                            scale*box_msg.box_dims.y,
                            scale*box_msg.box_dims.z]
        msg.shapes.append(shape)
        msg.poses.append(box_msg.pose.pose)
        return msg

class CircularPusher(Pusher):
    def __init__(self, planner, robot_state, 
                 traj_points = 10, 
                 traj_z_offset = 0.16,
                 angles_resolution = math.pi/12.0):
        super(CircularPusher, self).__init__(planner, robot_state)
        self.traj_points = traj_points
        self.traj_z_offset = traj_z_offset
        self.angles_resolution = angles_resolution
        
    def get_pushing_angles(self, box_msg):
        #downward gripper
        euler_angles = (0., math.pi/2., math.pi/2.)
        angles = transformations.quaternion_from_euler(euler_angles[0],
                                                       euler_angles[1],
                                                       euler_angles[2])
        return angles
    
class RightArmCircularPusher(CircularPusher):
    def __init__(self, planner, robot_state, 
                 traj_points = 10, 
                 traj_z_offset = 0.16,
                 angles_resolution = math.pi/12.0):
        super(RightArmCircularPusher, self).__init__(planner, 
                                             robot_state,
                                             traj_points,
                                             traj_z_offset,
                                             angles_resolution)
        
    def get_pushing_poses(self, box_msg):
        boxpose = (box_msg.pose.pose.position.x,
               box_msg.pose.pose.position.y,
               box_msg.pose.pose.position.z)
        boxangle = math.atan2(boxpose[1], boxpose[0])
        boxdist = math.sqrt(boxpose[0]*boxpose[0] + boxpose[1]*boxpose[1])
        
        angle_start = boxangle - math.pi/self.angles_resolution
        angle_end = boxangle + math.pi/self.angles_resolution
        poses = []
        all_angles = numpy.linspace(angle_start, angle_end, self.traj_points, endpoint=True)
        for alpha in all_angles:
            poses.append((boxdist * math.cos(alpha),
                          boxdist * math.sin(alpha),
                          boxpose[2] + self.traj_z_offset))
        return poses
    
    def which_arm(self):
        return "right_arm"

class LeftArmCircularPusher(CircularPusher):
    def __init__(self, planner, robot_state, 
                 traj_points = 10, 
                 traj_z_offset = 0.16,
                 angles_resolution = math.pi/12.0):
        super(LeftArmCircularPusher, self).__init__(planner, 
                                             robot_state,
                                             traj_points,
                                             traj_z_offset,
                                             angles_resolution)
    
    def get_pushing_poses(self, box_msg):
        boxpose = (box_msg.pose.pose.position.x,
               box_msg.pose.pose.position.y,
               box_msg.pose.pose.position.z)
        boxangle = math.atan2(boxpose[1], boxpose[0])
        boxdist = math.sqrt(boxpose[0]*boxpose[0] + boxpose[1]*boxpose[1])
        
        angle_start = boxangle + math.pi/self.angles_resolution
        angle_end = boxangle - math.pi/self.angles_resolution
        poses = []
        all_angles = numpy.linspace(angle_start, angle_end, self.traj_points, endpoint=True)
        for alpha in all_angles:
            poses.append((boxdist * math.cos(alpha),
                          boxdist * math.sin(alpha),
                          boxpose[2] + self.traj_z_offset))
        return poses
    
    def which_arm(self):
        return "left_arm"
    
class LateralPusher(Pusher):
    def __init__(self, planner, robot_state, 
                 traj_points, 
                 traj_z_offset,
                 linear_resolution):
        super(LateralPusher, self).__init__(planner, robot_state)
        self.traj_points = traj_points
        self.traj_z_offset = traj_z_offset
        self.linear_resolution = linear_resolution
        
    def get_pushing_angles(self, box_msg):
        #downward gripper
        euler_angles = (0., math.pi/2., math.pi/2.)
        angles = transformations.quaternion_from_euler(euler_angles[0],
                                                       euler_angles[1],
                                                       euler_angles[2])
        return angles
    
class LeftArmLateralPusher(LateralPusher):
    def __init__(self, planner, robot_state, 
                 traj_points = 10, 
                 traj_z_offset = 0.18,
                 linear_resolution = 0.20):
        super(LeftArmLateralPusher, self).__init__(planner, 
                                             robot_state,
                                             traj_points,
                                             traj_z_offset,
                                             linear_resolution)
    
    def get_pushing_poses(self, box_msg):
        boxpose = (box_msg.pose.pose.position.x,
               box_msg.pose.pose.position.y,
               box_msg.pose.pose.position.z)

        poses = []
        start_y = boxpose[1] + self.linear_resolution
        end_y = boxpose[1] - self.linear_resolution
        for y in numpy.linspace(start_y, end_y, self.traj_points, endpoint = True):
            poses.append((boxpose[0],
                          y,
                          boxpose[2] + self.traj_z_offset))
        return poses
    
    def which_arm(self):
        return "left_arm"
    
class RightArmLateralPusher(LateralPusher):
    def __init__(self, planner, robot_state, 
                 traj_points = 10, 
                 traj_z_offset = 0.18,
                 linear_resolution = 0.20):
        super(RightArmLateralPusher, self).__init__(planner, 
                                             robot_state,
                                             traj_points,
                                             traj_z_offset,
                                             linear_resolution)
    
    def get_pushing_poses(self, box_msg):
        boxpose = (box_msg.pose.pose.position.x,
               box_msg.pose.pose.position.y,
               box_msg.pose.pose.position.z)

        poses = []
        start_y = boxpose[1] - self.linear_resolution
        end_y = boxpose[1] + self.linear_resolution
        for y in numpy.linspace(start_y,end_y,self.traj_points, endpoint = True):
            poses.append((boxpose[0],
                          y,
                          boxpose[2] + self.traj_z_offset))
        return poses
    
    def which_arm(self):
        return "right_arm"
