import tabletop_actions
import rospy

from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest

import dynamic_reconfigure.client

import random

class ObjectDetector(object):
    def __init__(self):
        
        narrow_detector = "object_detection"        
        rospy.loginfo("waiting for %s service" % narrow_detector)
        self.narrow_detector =  rospy.ServiceProxy(narrow_detector, TabletopDetection)
        
        wide_detector = "wide_tabletop_segmentation"        
        rospy.loginfo("waiting for %s service" % wide_detector)
        self.wide_detector =  rospy.ServiceProxy(wide_detector, TabletopSegmentation)
        
        box_detector = "find_cluster_bounding_box"
        rospy.loginfo("waiting for %s service" % box_detector)
        self.box_detector =  rospy.ServiceProxy(box_detector, FindClusterBoundingBox)
        
        self.last_wide_msg = None
        self.last_narrow_msg = None
        self.last_box_msg = None
        
    def detect_narrow(self):
        req = TabletopDetectionRequest()
        req.return_clusters = 1
        req.return_models = 0        
        try:
            self.last_narrow_msg = self.narrow_detector(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling object_detection: %s"%e)
            return None
        if self.last_narrow_msg.detection.result != self.last_narrow_msg.detection.SUCCESS:
            self.last_narrow_msg = None
        if len(self.last_narrow_msg.detection.clusters) == 0:
            self.last_narrow_msg = None
        return self.last_narrow_msg
    
    def detect_wide(self):
        req = TabletopSegmentationRequest()
        try:
            self.last_wide_msg = self.wide_detector(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling object_detection: %s"%e)
            return None
        if self.last_wide_msg.result != self.last_wide_msg.SUCCESS:
            self.last_wide_msg
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
    
    def detect_bounding_box(self, cluster = None, use_random = False):
        
        if use_random:
            finder = self.find_random_cluster
        else:
            finder = self.find_biggest_cluster 
        
        if cluster is None:
            rospy.loginfo("Trying the narrow stereo...")
            res_narrow = self.detect_narrow()
            if res_narrow is None:
                rospy.logwarn("No luck with narrow stereo, trying the wide one")
                res_wide = self.detect_wide()
                if res_wide is None:
                    return None
                else:
                    cluster = finder(res_wide.clusters)
            else:
                cluster = finder(res_narrow.detection.clusters)
        
        if cluster is None: #even after the perception!
            rospy.logerr("No way there is an object in front of me!")
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
            res = self.detect_wide()
            if res.result != res.SUCCESS:
                rospy.logerr("No object found!")
                return False
            clusters = res.clusters 
            
            if use_random:
                object_cluster = self.find_random_cluster(clusters)
            else:
                object_cluster = self.find_biggest_cluster(clusters) 
            box_msg = self.detect_bounding_box(object_cluster)
        
        if box_msg is None:
            return False
        position = (box_msg.pose.pose.position.x,
                    box_msg.pose.pose.position.y,
                    box_msg.pose.pose.position.z - 0.5)
        frame = box_msg.pose.header.frame_id
        mover.point_head_to(position, frame)
        return True 
        
    def switch_narrow_projector(self, on = True):
        client = dynamic_reconfigure.client.Client("camera_synchronizer_node")
        
        if on:
            rospy.loginfo("Switching the projector on")
            params = {"projector_mode": 3, "narrow_stereo_trig_mode" :5}
        else:
            rospy.loginfo("Switching the projector off")
            params = {"projector_mode": 1}
        client.update_configuration(params)
