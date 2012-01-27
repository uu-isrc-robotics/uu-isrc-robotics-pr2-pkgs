#keeps trak of the displacement of an object-table before
#and after performing a movement. All the coordinates are
#expressed in the robot base_frame.
#movements are a displacement

Header header

#the object position in the base_frame, before moving
geometry_msgs/Point pre_movement_object_pose
#the table message, before moving
tabletop_object_detector/Table pre_movement_table

#the performed displacement
float64 dx
float64 dy
float64 dtheta
 
#the object position in the base_frame, after moving
geometry_msgs/Point post_movement_object_pose
#the table message, after moving
tabletop_object_detector/Table post_movement_table

