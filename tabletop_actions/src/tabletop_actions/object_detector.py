#import tabletop_actions
import rospy

from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
#from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest

import random
from rospy.service import ServiceException

class ObjectDetector(object):
    def __init__(self):
        
        narrow_detector = "object_detection"        
        rospy.loginfo("waiting for %s service" % narrow_detector)
        rospy.wait_for_service(narrow_detector)
        self.narrow_detector =  rospy.ServiceProxy(narrow_detector, TabletopDetection)
        
        wide_detector = "wide_object_detection"        
        rospy.loginfo("waiting for %s service" % wide_detector)
        rospy.wait_for_service(wide_detector)
        self.wide_detector =  rospy.ServiceProxy(wide_detector, TabletopDetection)
        
        box_detector = "find_cluster_bounding_box"
        rospy.loginfo("waiting for %s service" % box_detector)
        rospy.wait_for_service(box_detector)
        self.box_detector =  rospy.ServiceProxy(box_detector, FindClusterBoundingBox)
        
        collision_processing = "/tabletop_collision_map_processing/tabletop_collision_map_processing"
        rospy.loginfo("Waiting for collision processing service to come up") 
        rospy.wait_for_service(collision_processing)
        self.collision_processing = rospy.ServiceProxy(collision_processing, TabletopCollisionMapProcessing)
        
        self.last_wide_msg = None
        self.last_narrow_msg = None
        self.last_box_msg = None
        self.last_collision_processing_msg = None

        rospy.loginfo("ObjectDetector is ready")

    
    def __detect(self, detector):
        req = TabletopDetectionRequest()
        req.return_clusters = True
        req.return_models = True   
        try:
            reply = detector(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling object_detection: %s"%e)
            return None
        if reply.detection.result != reply.detection.SUCCESS:
            return None
        if len(reply.detection.clusters) == 0:
            return None
        return reply
        
    def detect_narrow(self):        
        self.last_narrow_msg = self.__detect(self.narrow_detector)
        return self.last_narrow_msg
    
    def detect_wide(self):
        self.last_wide_msg = self.__detect(self.wide_detector)        
        return self.last_wide_msg
    
    def find_biggest_cluster(self, clusters = None):
        if clusters is None:
            res = self.detect_narrow()
            if res is None:
                return None
            clusters = res.detection.clusters
         
        if len(clusters) == 0:
            rospy.logerr("No object found!")
            return
        
        #finding the biggest cluster
        max_len = 0
        index = 0    
        for i,c in enumerate(clusters):
            if len(c.points) > max_len:
                max_len = len(c.points)
                index = i
        object_cluster = clusters[index] 
        rospy.loginfo("Using object %d with %d points"%(index, len(object_cluster.points))) 
        return object_cluster
    
    def find_random_cluster(self, clusters = None):
        if clusters is None:
            res = self.detect_narrow()
            if res is None:
                return None
            clusters = res.detection.clusters
         
        if len(clusters) == 0:
            rospy.logerr("No object found!")
            return None
        
        #finding the biggest cluster
        
        index = random.randint(0, len(clusters)-1)
        object_cluster = clusters[index] 
        rospy.loginfo("Using object %d with %d points"%(index, len(object_cluster.points))) 
        return object_cluster
    
    def call_collision_map_processing(self, detection_result):
        
        if detection_result is None:
            rospy.logerr("Error: using a None detection_result")
            return None
        
        rospy.loginfo("Calling collision map processing")
        processing_call = TabletopCollisionMapProcessingRequest()    
        processing_call.detection_result = detection_result.detection
    #    ask for the exising map and collision models to be reset
        processing_call.reset_attached_models = True
        processing_call.reset_collision_models = True
        processing_call.reset_static_map = True
    #    after the new models are added to the environment
        processing_call.take_static_collision_map = True
        processing_call.desired_frame = "base_link"
        
        try:
            self.last_collision_processing_msg = self.collision_processing.call(processing_call)
        except ServiceException, e:
            rospy.logerr("Error calling collision map: %s" % str(e))
            self.last_collision_processing_msg = None
        return self.last_collision_processing_msg
    
    def try_to_detect(self):
        rospy.loginfo("Trying the narrow stereo...")
        res_narrow = self.detect_narrow()
        if res_narrow is None:
            rospy.logwarn("No luck with narrow stereo, trying the wide one")
            res_wide = self.detect_wide()
            return res_wide
        else:
            return res_narrow

    
    def detect_bounding_box(self, cluster = None, use_random = False, detection_result = None):        
        if use_random:
            finder = self.find_random_cluster
        else:
            finder = self.find_biggest_cluster 
        
        if cluster is None:            
            detection_result = self.try_to_detect()
            if detection_result is None:
                rospy.logerr("No way there is an object in front of me!")
                self.last_box_msg = None
                return None
            cluster = finder(detection_result.detection.clusters)
        
        if self.call_collision_map_processing(detection_result) is None:
            return None
        
        req = FindClusterBoundingBoxRequest()
        req.cluster = cluster
        try:
            self.last_box_msg = self.box_detector(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)
            return None
        if not self.last_box_msg.error_code:
            return self.last_box_msg
        else:
            rospy.logwarn("An error was reported when trying tabletop")
            return self.last_box_msg
    
    def point_head_at(self, mover, box_msg = None, use_random = False):
        if box_msg is None:
#            res = self.detect_wide()
            res = self.try_to_detect()
            if res is None:
                rospy.logerr("No object found!")
                return False
            clusters = res.detection.clusters 
            
            if use_random:
                object_cluster = self.find_random_cluster(clusters)
            else:
                object_cluster = self.find_biggest_cluster(clusters) 
            box_msg = self.detect_bounding_box(cluster = object_cluster, 
                                               detection_result = res)
        
        if box_msg is None:
            return False
        position = (box_msg.pose.pose.position.x,
                    box_msg.pose.pose.position.y,
                    box_msg.pose.pose.position.z - 0.5)
        frame = box_msg.pose.header.frame_id
        mover.point_head_to(position, frame)
        return True 
