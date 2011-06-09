import rospy
from visualization_msgs.msg import Marker
from mapping_msgs.msg import CollisionObject
from geometry_msgs.msg import Point
from geometric_shapes_msgs.msg import Shape
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
                                                              latch=True)
        self.collision_objects_pub = rospy.Publisher("collision_object", CollisionObject)
        self.robot_state = robot_state
        self.mover = pr2_control_utilities.PR2JointMover(robot_state)
        
        if pre_pushing_pose is None:
            self.pre_pushing_pose = pr2_control_utilities.PR2JointMover(self.robot_state)
            self.pre_pushing_pose.store_targets()
        else:
            self.pre_pushing_pose  = pre_pushing_pose
        
    def push_object(self, box_msg, ignore_errors = False):
        
        if self.which_arm() == "right_arm":
            move_arm = self.planner.move_right_arm
            move_arm_trajectory_non_collision = self.planner.move_right_arm_trajectory_non_collision
            trajectory_creator = self.planner.create_right_arm_trjectory_non_collision
        elif self.which_arm() == "left_arm":
            move_arm_trajectory_non_collision = self.planner.move_left_arm_trajectory_non_collision
            move_arm = self.planner.move_left_arm
            trajectory_creator = self.planner.create_left_arm_trjectory_non_collision
        else:
            return self.__getout("Non existing arm being used!", self.pre_pushing_pose)
        
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
                                 ignore_errors=ignore_errors)
        if res:
            rospy.loginfo("Pushing trajectory is ok")
            trajectory, times, vels = res
        else:
            return self.__getout("The trajectory is not feasible", self.pre_pushing_pose)
        
        
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
            return self.__getout("Pre-push planning returned with an error", 
                                 self.pre_pushing_pose)
        
        
        self.planner.joint_mover.time_to_reach = 5.0
        self.mover.set_head_state(initial_head_position)
        rospy.loginfo("Starting the push")
        self.mover.execute_trajectory(trajectory, times, vels, self.which_arm(), True)
        
        rospy.loginfo("Moving back to the original place")
        self.pre_pushing_pose.time_to_reach = 10
        self.pre_pushing_pose.execute_and_wait()
        
        return True
    
    def __getout(self, msg, mover = None):
        rospy.logerr(msg)
        if mover is not None:
            mover.execute_and_wait()
        return False
    
    def get_pushing_angles(self, box_msg):
        raise exceptions.NotImplementedError("Do not use a Pusher class directly!")
    
    def get_pushing_poses(self, box_msg):
        raise exceptions.NotImplementedError("Do not use a Pusher class directly!")
    
    def which_arm(self):
        raise exceptions.NotImplementedError("Do not use a Pusher class directly!")
    
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

class RightArmPusher(Pusher):
    def __init__(self, planner, robot_state, 
                 traj_points = 10, 
                 traj_z_offset = 0.2):
        super(RightArmPusher, self).__init__(planner, robot_state)
        
        self.traj_points = traj_points
        self.traj_z_offset = traj_z_offset
    
    def get_pushing_angles(self, box_msg):
        #downward gripper
        euler_angles = (0., math.pi/2., math.pi/2.)
        angles = transformations.quaternion_from_euler(euler_angles[0],
                                                       euler_angles[1],
                                                       euler_angles[2])
        return angles
    
    def get_pushing_poses(self, box_msg):
        boxpose = (box_msg.pose.pose.position.x,
               box_msg.pose.pose.position.y,
               box_msg.pose.pose.position.z)
        boxangle = math.atan2(boxpose[1], boxpose[0])
        boxdist = math.sqrt(boxpose[0]*boxpose[0] + boxpose[1]*boxpose[1])
        
        angle_start = boxangle - math.pi/12.0
        angle_end = boxangle + math.pi/12.0
        poses = []
        all_angles = numpy.linspace(angle_start, angle_end, self.traj_points, endpoint=True)
        for alpha in all_angles:
            poses.append((boxdist * math.cos(alpha),
                          boxdist * math.sin(alpha),
                          boxpose[2] + self.traj_z_offset))
        return poses
    
    def which_arm(self):
        return "right_arm"

class LeftArmPusher(Pusher):
    def __init__(self, planner, robot_state, 
                 traj_points = 10, 
                 traj_z_offset = 0.2):
        super(LeftArmPusher, self).__init__(planner, robot_state)
        
        self.traj_points = traj_points
        self.traj_z_offset = traj_z_offset
    
    def get_pushing_angles(self, box_msg):
        #downward gripper
#        euler_angles = (0.,0., 0)
        euler_angles = (0., math.pi/2., math.pi/2)
        angles = transformations.quaternion_from_euler(euler_angles[0],
                                                       euler_angles[1],
                                                       euler_angles[2])
        return angles
    
    def get_pushing_poses(self, box_msg):
        boxpose = (box_msg.pose.pose.position.x,
               box_msg.pose.pose.position.y,
               box_msg.pose.pose.position.z)
        boxangle = math.atan2(boxpose[1], boxpose[0])
        boxdist = math.sqrt(boxpose[0]*boxpose[0] + boxpose[1]*boxpose[1])
        
        angle_start = boxangle + math.pi/12.0
        angle_end = boxangle - math.pi/12.0
        poses = []
        all_angles = numpy.linspace(angle_start, angle_end, self.traj_points, endpoint=True)
        for alpha in all_angles:
            poses.append((boxdist * math.cos(alpha),
                          boxdist * math.sin(alpha),
                          boxpose[2] + self.traj_z_offset))
        return poses
    
    def which_arm(self):
        return "left_arm"