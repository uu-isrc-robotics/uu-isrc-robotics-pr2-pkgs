"""
Copyright (c) 2012, Lorenzo Riano.
All rights reserved.

Various common utilities
"""

from interactive_markers.interactive_marker_server import InteractiveMarkerControl
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import numpy
import rospy
import copy

def make_6DOF_marker(int_marker):
    """
    Creates a 6DOF InteractiveMarkerContro that can be translated and rotated.

    Parameters:
    int_marker: a previously created InteractiveMarker to attach the new marker
    to.
    """
    #x movement
    control = InteractiveMarkerControl()
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    int_marker.controls.append(control);

    #x rotation
    control = InteractiveMarkerControl()
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    int_marker.controls.append(control);

    #y movement
    control = InteractiveMarkerControl()
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    int_marker.controls.append(control);

    #y rotation
    control = InteractiveMarkerControl()
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    int_marker.controls.append(control);

    #z movement
    control = InteractiveMarkerControl()
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    int_marker.controls.append(control);

    #z rotation
    control = InteractiveMarkerControl()
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    int_marker.controls.append(control);

def axis_marker(pose_stamped, id,
                ns):
    """
    Create three Marker msgs from a PoseStamped set of arrows (x=red, y=green, z=blue).
    
    Parameters:
    id: the id number for the x-arrow (y is id+1, z is id+2)
    ns: the namespace for the marker
    """
    marker = Marker()
    marker.header = pose_stamped.header
    marker.ns = ns
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.01
    marker.scale.y = 0.02
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(1/5.0)

    orientation = pose_stamped.pose.orientation
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    mat = tf.transformations.quaternion_matrix(quat)
    start = (pose_stamped.pose.position.x, 
             pose_stamped.pose.position.y, 
             pose_stamped.pose.position.z)
    x_end = list(mat[:,0][0:3]*.05 + numpy.array(start))
    y_end = list(mat[:,1][0:3]*.05 + numpy.array(start))
    z_end = list(mat[:,2][0:3]*.05 + numpy.array(start))
    
    marker.id = id
    marker.points = [pose_stamped.pose.position, Point(*x_end)]
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    marker2 = copy.deepcopy(marker)
    marker2.id = id+1
    marker2.points = [pose_stamped.pose.position, Point(*y_end)]
    marker2.color.r = 0.0
    marker2.color.g = 1.0
    marker2.color.b = 0.0
    
    marker3 = copy.deepcopy(marker2)
    marker3.id = id+2
    marker3.points = [pose_stamped.pose.position, Point(*z_end)]
    marker3.color.r = 0.0
    marker3.color.g = 0.0
    marker3.color.b = 1.0

    return (marker, marker2, marker3)
