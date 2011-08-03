#import tabletop_actions
import rospy

from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
#from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest

import random
from rospy.service import ServiceException
from visualization_msgs.msg import Marker
import math

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
        
        self.box_drawer = rospy.Publisher("box_drawer", 
                                          Marker
                                          )
        
        self.last_wide_msg = None
        self.last_narrow_msg = None
        self.last_detection_msg = None
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
        self.last_detection_msg = self.last_narrow_msg
        return self.last_narrow_msg
    
    def detect_wide(self):
        self.last_wide_msg = self.__detect(self.wide_detector)
        self.last_detection_msg = self.last_wide_msg        
        return self.last_wide_msg
    
    def find_biggest_cluster(self, clusters):
        if clusters is None:
            return None
         
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
    
    def find_random_cluster(self, clusters):
        if clusters is None:
            return None
         
        if len(clusters) == 0:
            rospy.logerr("No object found!")
            return None
        
        #using a random cluster        
        index = random.randint(0, len(clusters)-1)
        object_cluster = clusters[index] 
        rospy.loginfo("Using object %d with %d points"%(index, len(object_cluster.points))) 
        return object_cluster
    
    def find_closest_cluster(self, clusters):
        '''
        Searches for the closest cluster among clusters. It uses 
        detect_bounding_box to find the cluster position
        @param clusters: a list of PointCloud among wich to find the closest 
        cluster 
        '''
        if clusters is None:
            return None
         
        if len(clusters) == 0:
            rospy.logerr("No object found!")
            return None
        
        boxes = []
        for cluster in clusters:
            box_msg = self.detect_bounding_box(cluster)
            if box_msg is not None:
                boxes.append(box_msg)
        
        if len(boxes) == 0:
            return None
        
        closest_index = 0
        closest_dist = 10000
        
        for i, box in enumerate(boxes):
            dist = math.sqrt(box.pose.pose.position.x**2 +
                             box.pose.pose.position.y**2 +
                             box.pose.pose.position.z**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i
        
        rospy.loginfo("Using object %d with %f distance"%
                      (closest_index, closest_dist))
        return clusters[closest_index]
        
    
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
            self.detect_wide()
        if self.last_detection_msg is not None:
            self.call_collision_map_processing(self.last_detection_msg)
        return self.last_detection_msg

    
    def detect_bounding_box(self, cluster = None, 
                            cluster_choser = "find_random_cluster"):
        try:
            finder = self.__getattribute__(cluster_choser)
        except AttributeError:
            rospy.logwarn("Cluster choser %s does not exist, using the random one" %
                          cluster_choser)
            finder = self.find_random_cluster
        
        if cluster is None:            
            detection_result = self.try_to_detect()
            if detection_result is None:
                rospy.logerr("No way there is an object in front of me!")
                self.last_box_msg = None
                return None
            cluster = finder(detection_result.detection.clusters)
        
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
    
    def point_head_at(self, mover, box_msg = None, 
                      cluster_choser = "find_random_cluster"):
        if box_msg is None:
#            res = self.detect_wide()
            res = self.try_to_detect()
            if res is None:
                rospy.logerr("No object found!")
                return False
            clusters = res.detection.clusters 
            
            try:
                finder = self.__getattribute__(cluster_choser)
            except AttributeError:
                rospy.logwarn("Cluster choser %s does not exist, using the random one" %
                              cluster_choser)
                finder = self.find_random_cluster

            object_cluster = finder(clusters) 
            box_msg = self.detect_bounding_box(cluster = object_cluster)
        
        if box_msg is None:
            return False
        position = (box_msg.pose.pose.position.x,
                    box_msg.pose.pose.position.y,
                    box_msg.pose.pose.position.z)
        frame = box_msg.pose.header.frame_id
        mover.point_head_to(position, frame)
        return True 

    def draw_bounding_box(self, id, box_msg, color = (1.0, 1.0, 0.0, 0.0)):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "object_detector"
        marker.type = Marker.CUBE
        marker.action = marker.ADD
        marker.id = id
        marker.header.frame_id = box_msg.pose.header.frame_id
        
        marker.pose = box_msg.pose.pose
        marker.scale.x = box_msg.box_dims.x
        marker.scale.y = box_msg.box_dims.y
        marker.scale.z = box_msg.box_dims.z
        
        marker.color.a = color[0]
        marker.color.r = color[1]
        marker.color.g = color[2]
        marker.color.b = color[3]
        
        self.box_drawer.publish(marker)

    def search_for_object(self, mover, trials = 1, 
                          cluster_choser="find_random_cluster", 
                          max_pan=0.4, min_pan=-0.4,
                          max_tilt = 1.1, min_tilt = 0.8):    

        #first try without moving        
        if self.point_head_at(mover,cluster_choser = cluster_choser):
            return True
        trials -= 1
        while trials > 0:
            pan = random.uniform(min_pan, max_pan)
            tilt = random.uniform(min_tilt, max_tilt)
            mover.set_head_state((pan, tilt))
            rospy.sleep(0.5)
            
            if self.point_head_at(mover,cluster_choser = cluster_choser):
                return True
            trials -= 1
        return False
            
            
