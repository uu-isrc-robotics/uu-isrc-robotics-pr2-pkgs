import roslib 
roslib.load_manifest('learn_actions')
import rospy
import tf
from learn_actions.msg import ObjectDiscovery
from learn_actions.msg import ObjectTableChangesWithMovement
from geometry_msgs.msg import Point
from pr2_control_utilities import utils
import copy

class LogPushingResult(object):
    """
    Searches for an object and tries to push it, emitting a corresponging
    ObjectDiscovery msg.
    It stores the object in the specified frame of reference, which should 
    be external to the robot, so that it can try to detect it again with
    subsequent calls.
    """

    def __init__(self, detector, pusher_l, pusher_r, joint_mover,
            base_mover,
            static_frame = "odom_combined",
            tf_listener = None):
        """
        
        Parameters:
        detector: an ObjectDetector instance
        pusher_l: a LeftArmPusher instance
        pusher_r: a RightArmPusher instance
        joint_mover: a Pr2JointMover instance
        base_mover: a PR2BaseMover instance
        static_frame: a frame independent from the robot motion
        tf_listener: a TransformListener
        """
        self.detector = detector
        self.pusher_l = pusher_l
        self.pusher_r = pusher_r
        self.joint_mover = joint_mover
        self.base_mover = base_mover
        self.static_frame = static_frame
    
        self.object_pose = None #it will become a triplet
        if tf_listener is None:
            self.tf_listner = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
    
        self.obj_discovery_pub = rospy.Publisher("object_discovery", 
                ObjectDiscovery)
        self.obj_changes_pub = rospy.Publisher("object_changes", 
                ObjectTableChangesWithMovement)

        #initializing the fillin strucure
        self.fillin_changes_msg = ObjectTableChangesWithMovement()
        self.fillin_changes_msg.pre_movement_object_pose = None
        self.fillin_changes_msg.pre_movement_table = None
        self.prev_trans, self.prev_rot = base_mover.current_position(
                frame=self.static_frame)
        self.fillin_changes_msg.dx = 0.
        self.fillin_changes_msg.dy = 0.
        self.fillin_changes_msg.dtheta = 0.
        self.fillin_changes_msg.post_movement_object_pose = None
        self.fillin_changes_msg.post_movement_table = None

        rospy.loginfo("LogPushingResult is ready")

    def search_object(self):
        """
        Searches for an object using the detector.

        If the object had already be found before, it steers the head
        towards the last known location.

        Returns:
        True if the object is found, False otherwise
        """
        if self.object_pose is not None:
            self.joint_mover.time_to_reach = 0.5
            self.joint_mover.point_head_to(self.object_pose, self.static_frame)
            rospy.loginfo("Pointing the head towards %s", self.object_pose)
            #I don't know if the action has completed!
            rospy.sleep(0.5)
            
        
        head_pan, head_tilt = self.joint_mover.robot_state.head_positions
        
        res = self.detector.search_for_object(self.joint_mover,
                trials = 5,
                max_pan = head_pan + 0.2,
                min_pan = head_pan - 0.2,
                max_tilt = head_tilt + 0.1,
                min_tilt = head_tilt - 0.1,
                cluster_choser = "find_closest_cluster"
                )

        if res:
            rospy.loginfo("Object found, storing its transformation in the frame %s",
                    self.static_frame)
            box_pose = self.detector.last_box_msg.pose
            self.tf_listener.waitForTransform(self.static_frame,
                    box_pose.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1)
                    )
            pos = self.tf_listener.transformPose(self.static_frame, box_pose)
            self.object_pose = (pos.pose.position.x,
                           pos.pose.position.y,
                           pos.pose.position.z
                          )

        return res


    def publish_object_changes(self):
        """
        Publishes the ObjectTableChangesWithMovement msg, unless one of the
        fields is None. Also it rotates the premotion and postmotion fields,
        and it updates the displacement fields.

        Sends the new message unless any of the premovement field is None
        Parameters:
        """

        #the premovement is updated with the previous postmovement
        self.fillin_changes_msg.pre_movement_object_pose = copy.copy(
                self.fillin_changes_msg.post_movement_object_pose)
        self.fillin_changes_msg.pre_movement_table = copy.copy(
                self.fillin_changes_msg.post_movement_table)


        #the postmovement is updated with the current reading
        box = self.detector.last_box_msg
        table = self.detector.last_detection_msg.detection.table

        newpos = Point()
        newpos.x = box.pose.pose.position.x
        newpos.y = box.pose.pose.position.y
        newpos.z = box.pose.pose.position.z

        self.fillin_changes_msg.post_movement_object_pose = newpos
        self.fillin_changes_msg.post_movement_table = table

        #the displacement is updated
        trans, rot = self.base_mover.current_position(frame=self.static_frame)
        rospy.loginfo("Current trans: %s, rot: %s", trans, rot)

        robot_trans = utils.convert_point(self.tf_listener,
                                          (self.prev_trans[0],
                                           self.prev_trans[1],
                                           0
                                          ),
                                          self.static_frame,
                                          "/base_link"
                                         )

        dtheta = rot[2] - self.prev_rot[2]

        self.fillin_changes_msg.dx =  -robot_trans[0]
        self.fillin_changes_msg.dy = -robot_trans[1]
        self.fillin_changes_msg.dtheta = dtheta
        self.prev_trans = trans
        self.prev_rot = rot

        #unless any of the premovement fields is None, send the message
        if (self.fillin_changes_msg.pre_movement_object_pose is not None and
                self.fillin_changes_msg.pre_movement_table is not None):
            rospy.loginfo("Ready to publish!")
            self.obj_changes_pub.publish(self.fillin_changes_msg)
        else:
            rospy.loginfo("Not publishing yet, the premovement is None")
        
    def publish_object_discovery(self, box, table, whicharm, torso_joint):
        """
        Compose an ObjectDiscovery message out of the various parameters.

        Parameters:
        box: a FindClusterBoundingBox response
        table: a tabletop_object_detector/Table msg
        whicharm: see ObjectDiscovery
        torso_joint: the current value of the PR2 torso joint
        """
        
        msg = ObjectDiscovery()
        msg.header.stamp = rospy.Time.now()

        msg.object_pose.x = box.pose.pose.position.x
        msg.object_pose.y = box.pose.pose.position.y
        msg.object_pose.z = box.pose.pose.position.z
        
        msg.table = table
        
        msg.grasping_result = whicharm
        msg.torso_joint = torso_joint

        self.obj_discovery_pub.publish(msg) 
    
    def search_and_detect_changes_publisher(self):
        """
        Searches for an object and, if it's found, checks if it can push it.
        """
        if not self.search_object():
            rospy.logwarn("Object not found, no publishing will be done")
            return

        self.publish_object_changes()


    def try_push_and_publish(self):
        """
        Searches for an object and, if it's found, checks if it can push it.
        It then publishes an ObjectDiscovery accordingly.
        """

        if not self.search_object():
            rospy.logwarn("Object not found, no publishing will be done")
            return

        self.publish_object_changes()

        box = self.detector.last_box_msg
        table = self.detector.last_detection_msg.detection.table
        
        rospy.loginfo("Trying the left pusher")
        ret =  self.pusher_l.test_push(box)
        if ret is not None:
            rospy.loginfo("Pushing with left arm is ok")
            self.publish_object_discovery(box,  table,
                           ObjectDiscovery.LEFT_ARM,
                           self.joint_mover.robot_state.torso_position[0]
                          )
            return "success"
        else: 
            rospy.loginfo("Trying the right pusher")
            ret =  self.pusher_r.test_push(box)
            if ret is not None:
                rospy.loginfo("Pushing with right arm is ok")
                self.publish_object_discovery(box,  table,
                           ObjectDiscovery.RIGHT_ARM,
                           self.joint_mover.robot_state.torso_position[0]
                          )
                return "success"
            else:
                rospy.logerr("Pushing is not feasible")
                self.publish_object_discovery(box,  table,
                           ObjectDiscovery.FAIL,
                           self.joint_mover.robot_state.torso_position[0]
                          )
                return "failure" 

