import roslib
roslib.load_manifest('tabletop_actions')
import rospy

from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest
from mapping_msgs.msg import CollisionObject
from geometric_shapes_msgs.msg import Shape
from motion_planning_msgs.msg import OrderedCollisionOperations, CollisionOperation
from motion_planning_msgs.msg import AllowedContactSpecification

def call_object_segmentation(srv = "/wide_tabletop_segmentation"):
    req = TabletopSegmentationRequest()
    service_name = srv
    rospy.loginfo("waiting for %s service" % srv)
    rospy.wait_for_service(service_name)
    serv = rospy.ServiceProxy(service_name, TabletopSegmentation)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling object_detection: %s"%e)
        return None
    return res

def call_object_detector(srv = "/object_detection"):

    req = TabletopDetectionRequest()
    req.return_clusters = 1
    req.return_models = 0
    service_name = srv
    rospy.loginfo("waiting for %s service" % srv)
    serv = rospy.ServiceProxy(service_name, TabletopDetection)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling object_detection: %s"%e)
        return None
    return res


#call plan_point_cluster_grasp to get candidate grasps for a cluster
def call_find_cluster_bounding_box(cluster):

    req = FindClusterBoundingBoxRequest()
    req.cluster = cluster
    service_name = "find_cluster_bounding_box"
    rospy.loginfo("waiting for find_cluster_bounding_box service")
    rospy.wait_for_service(service_name)
    rospy.loginfo("service found")
    serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox)    
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)
        return None
    if not res.error_code:
        return res
    else:
        rospy.logwarn("An error was reported when trying tabletop")
        return res
    


def build_object_from_box(box_msg, id):
    msg = CollisionObject()
    
#    vertices = build_box_vertices(box_msg)
    msg.header.frame_id = box_msg.pose.header.frame_id
    msg.header.stamp = rospy.Time.now()
    msg.id = id
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

def build_allowed_contact_specification(box_msg):
    msg = AllowedContactSpecification()
    msg.name = "grasping_object_region"
    shape = Shape()
    shape.type = shape.BOX
    shape.dimensions = [box_msg.box_dims.x,
                        box_msg.box_dims.y,
                        box_msg.box_dims.z]
    msg.shape = shape
    msg.pose_stamped = box_msg.pose
    
    msg.link_names = ["r_gripper_palm_link",
                      "r_gripper_l_finger_link", 
                      "r_gripper_r_finger_link",
                      "r_gripper_l_finger_tip_link",
                      "r_gripper_r_finger_tip_link"]
    msg.penetration_depth = 10.5
    return msg
    
def build_collision_operations(object1, object2):
    msg = OrderedCollisionOperations()
    collision = CollisionOperation()
    
    collision.operation = CollisionOperation.DISABLE
#    collision.penetration_distance = 1.0
    collision.object1 = object1
    collision.object2 = object2
    msg.collision_operations.append(collision)
    return msg
    
    
def point_head_at(mover, box_msg = None):
    if box_msg is None:
        res = call_object_segmentation()
        if res.result != res.SUCCESS:
            rospy.logerr("No object found!")
            return
        clusters = res.clusters 
        
        #finding the biggest cluster
        max_len = 0
        index = 0    
        for i,c in enumerate(clusters):
            if len(c.points) > max_len:
                max_len = len(c.points)
                index = i 
        
        object_cluster = clusters[index] 
        rospy.loginfo("Using object %d with %d points"%(index, len(object_cluster.points)))
        box_msg = call_find_cluster_bounding_box(object_cluster)
    
    position = (box_msg.pose.pose.position.x,
                box_msg.pose.pose.position.y,
                box_msg.pose.pose.position.z - 0.45)
    frame = box_msg.pose.header.frame_id
    mover.point_head_to(position, frame)

