Header header
#the object position in the base_frame.
geometry_msgs/Point object_pose
#the table message
tabletop_object_detector/Table table
 
#the torso joint
float64 torso_joint
 
#the grasping result
byte FAIL=0
byte LEFT_ARM=1
byte RIGHT_ARM=2
byte grasping_result

