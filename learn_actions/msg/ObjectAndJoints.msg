#keeps trak of the displacement of an object-table before
#and after performing a pushing action. All the coordinates are
#expressed in the robot base_frame.
#The pushing action is represented as an array of Poses

Header header

#the objects bounding boxes
geometry_msgs/Vector3[] box_dims

#the objects position
geometry_msgs/PoseStamped[] object_poses

string[] object_names

geometry_msgs/PoseStamped left_gripper_pose
geometry_msgs/PoseStamped right_gripper_pose

sensor_msgs/JointState joints

