import roslib 
roslib.load_manifest('learn_actions')
import rospy
#import pr2_control_utilities
#import tabletop_actions.object_detector as object_detector
#import tabletop_actions.pushers as pushers
#from learn_actions.msg import ObjectDiscovery
import tf
from learn_actions.msg import ObjectDiscovery

class LogPushingResult(object):
    """
    Searches for an object and tries to push it, emitting a corresponging
    ObjectDiscovery msg.
    It stores the object in the specified frame of reference, which should 
    be external to the robot, so that it can try to detect it again with
    subsequent calls.
    
    """

    def __init__(self, detector, pusher_l, pusher_r, joint_mover,
            static_frame = "odom_combined",
            tf_listener = None):
        """
        
        Parameters:
        detector: an ObjectDetector instance
        pusher_l: a LeftArmPusher instance
        pusher_r: a RightArmPusher instance
        joint_mover: a Pr2JointMover instance
        static_frame: a frame independent from the robot motion
        tf_listener: a TransformListener
        """
        self.detector = detector
        self.pusher_l = pusher_l
        self.pusher_r = pusher_r
        self.joint_mover = joint_mover
        self.static_frame = static_frame
    
        self.object_pose = None #it will become a triplet
        if tf_listener is None:
            self.tf_listner = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
    
        self.obj_discovery_pub = rospy.Publisher("object_discovery", 
                ObjectDiscovery)

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
                min_tilt = head_tilt - 0.1
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

    def publish_result(self, box, table, whicharm, torso_joint):
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
        
    def try_push_and_publish(self):
        """
        Searches for an object and, if it's found, checks if it can push it.
        It then publishes an ObjectDiscovery accordingly.
        """

        if not self.search_object():
            rospy.logwarn("Object not found, no publishing will be done")
            return

        box = self.detector.last_box_msg
        table = self.detector.last_detection_msg.detection.table
        
        rospy.loginfo("Trying the left pusher")
        ret =  self.pusher_l.test_push(box)
        if ret is not None:
            rospy.loginfo("Pushing with left arm is ok")
            self.publish_result(box,  table,
                           ObjectDiscovery.LEFT_ARM,
                           self.joint_mover.robot_state.torso_position[0]
                          )
            return "success"
        else: 
            rospy.loginfo("Trying the right pusher")
            ret =  self.pusher_r.test_push(box)
            if ret is not None:
                rospy.loginfo("Pushing with right arm is ok")
                self.publish_result(box,  table,
                           ObjectDiscovery.RIGHT_ARM,
                           self.joint_mover.robot_state.torso_position[0]
                          )
                return "success"
            else:
                rospy.logerr("Pushing is not feasible")
                self.publish_result(box,  table,
                           ObjectDiscovery.FAIL,
                           self.joint_mover.robot_state.torso_position[0]
                          )
                return "failure" 

