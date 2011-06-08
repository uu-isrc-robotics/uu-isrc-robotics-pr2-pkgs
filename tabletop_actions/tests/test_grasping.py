#! /usr/bin/python
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)

import rospy
import actionlib
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest
from object_manipulation_msgs.msg import PickupAction, PickupGoal
from object_manipulation_msgs.msg import PlaceAction

from rospy.exceptions import ROSException
import sys

if __name__=="__main__":
    rospy.init_node(PKG, anonymous=False)
    
    OBJECT_DETECTION_SERVICE_NAME = "/object_detection"
    COLLISION_PROCESSING_SERVICE_NAME = "/tabletop_collision_map_processing/tabletop_collision_map_processing"
    PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup"
    PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place"
    
    #object detection
    try:
        rospy.loginfo("Waiting for object detection service to come up") 
        rospy.wait_for_service(OBJECT_DETECTION_SERVICE_NAME, 5.0)
    except ROSException:        
        rospy.logerr("Service %s didn't come up!" % OBJECT_DETECTION_SERVICE_NAME)
        rospy.signal_shutdown("")
        sys.exit()
    
    object_detection_srv = rospy.ServiceProxy(OBJECT_DETECTION_SERVICE_NAME, TabletopDetection)
    
    #collision processing
    try:
        rospy.loginfo("Waiting for collision processing service to come up") 
        rospy.wait_for_service(COLLISION_PROCESSING_SERVICE_NAME, 5.0)
    except ROSException:        
        rospy.logerr("Service %s didn't come up!" % COLLISION_PROCESSING_SERVICE_NAME)
        rospy.signal_shutdown("")
        sys.exit()   
    collision_processing_srv = rospy.ServiceProxy(COLLISION_PROCESSING_SERVICE_NAME, TabletopCollisionMapProcessing)
    
    #pickup client
    pickup_client = actionlib.SimpleActionClient(PICKUP_ACTION_NAME, PickupAction)
    while not pickup_client.wait_for_server(rospy.Duration(2.0)):
        rospy.loginfo("Witing for action client %s", PICKUP_ACTION_NAME)
    
    #place client
    place_client = actionlib.SimpleActionClient(PLACE_ACTION_NAME, PlaceAction)
    while not place_client.wait_for_server(rospy.Duration(2.0)):
        rospy.loginfo("Witing for action client %s", PICKUP_ACTION_NAME)
        
    rospy.loginfo("Calling tabletop detector")
    detection_call = TabletopDetectionRequest()
    detection_call.return_clusters = True
    detection_call.return_models = True
    
    detection_reply = object_detection_srv.call(detection_call)    
    if len(detection_reply.detection.clusters) == 0:
        rospy.logerr("No objects found!")
        rospy.signal_shutdown("")
        sys.exit()   
    
        rospy.loginfo("%d objects detected" % len(detection_reply.detection.clusters))
#    detection_reply.detection.cluster_model_indices = tuple(xrange(len(detection_reply.detection.clusters)))

    rospy.loginfo("Calling collision map processing")
    processing_call = TabletopCollisionMapProcessingRequest()    
    processing_call.detection_result = detection_reply.detection
#    ask for the exising map and collision models to be reset
    processing_call.reset_attached_models = False
    processing_call.reset_collision_models = False
    processing_call.reset_static_map = False
#    after the new models are added to the environment
    processing_call.take_static_collision_map = False
    processing_call.desired_frame = "base_link"
    
    processing_reply = collision_processing_srv.call(processing_call)
    if len(processing_reply.graspable_objects) == 0:
        rospy.logerr("Collision map processing returned no graspable objects")
        rospy.signal_shutdown("")
        sys.exit()
    
    sys.exit()
    rospy.loginfo("Calling Pickup")
    
    max_len = 0
    index = 0
    
    for i,c in enumerate(detection_reply.detection.clusters):
        if len(c.points) > max_len:
            max_len = len(c.points)
            index = i 
    rospy.loginfo("Grasping object %d with %d points"%(index, len(detection_reply.detection.clusters[index].points)))
    
    pickup_goal = PickupGoal()
    pickup_goal.target = processing_reply.graspable_objects[index]
    pickup_goal.collision_object_name = processing_reply.collision_object_names[index]
    pickup_goal.collision_support_surface_name = processing_reply.collision_support_surface_name
    
    pickup_goal.arm_name = "left_arm"
    pickup_goal.desired_approach_distance = 0.2
    pickup_goal.min_approach_distance = 0.05
    
    pickup_goal.lift.direction.header.frame_id = "base_link"
    pickup_goal.lift.direction.vector.x = 0
    pickup_goal.lift.direction.vector.y = 0
    pickup_goal.lift.direction.vector.z = 1
    
#    request a vertical lift of 10cm after grasping the object
    pickup_goal.lift.desired_distance = 0.1
    pickup_goal.lift.min_distance = 0.05
#    do not use tactile-based grasping or tactile-based lift
    pickup_goal.use_reactive_lift = False
    pickup_goal.use_reactive_execution = False
    
    pickup_client.send_goal_and_wait(pickup_goal, rospy.Duration(120.0))
    