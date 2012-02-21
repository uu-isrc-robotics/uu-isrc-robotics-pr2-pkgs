#keeps trak of the displacement of an object-table before
#and after performing a pushing action. All the coordinates are
#expressed in the robot base_frame.
#The pushing action is represented as an array of Poses

Header header

#the object bounding box observed before the movement
geometry_msgs/Vector3 pre_movement_box_dims

#the object position in the base_frame, before moving
geometry_msgs/Pose pre_movement_object_pose

#the trajectory the end effector followed
geometry_msgs/PoseStamped[] trajectory
 
#the object bounding box observed after the movement
geometry_msgs/Vector3 post_movement_box_dims
#the object position in the base_frame, after moving
geometry_msgs/Pose post_movement_object_pose

#which arm was used for the pushing
uint8 LEFT=0
uint8 RIGHT=1
uint8 whicharm
