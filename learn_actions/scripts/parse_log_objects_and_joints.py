#! /usr/bin/python
import roslib
roslib.load_manifest("learn_actions")
import rospy
import rosbag
import numpy
import glob
import sys
import cPickle
from optparse import OptionParser
import itertools
import h5py
from learn_actions.msg import ObjectAndJoints
from geometry_msgs.msg import PoseStamped

def main():
    parser = OptionParser()
    parser.add_option("-i", "--input_file", dest="input_file",
                      help="input bag FILEs. Use wildcards for multiple files",
                      metavar="FILE", action="store")
    parser.add_option("-o", "--output_file", dest="output_file",
                      help="output FILE", metavar="FILE")

    (options, _) = parser.parse_args()

    if options.input_file is None:
        rospy.logerr("The input bag has to be specified")
        sys.exit()
    if options.output_file is None:
        rospy.logerr("The output file has to be specified")
        sys.exit()


    f = h5py.File(options.output_file, "w")

    gen = (glob.glob(f) for f in options.input_file.split())
    traj_num = 0
    for filename in itertools.chain.from_iterable(gen):

        rospy.loginfo("Opening bag %s"% filename)
        bag = rosbag.Bag(filename)

        l_traj_pos = []
        l_traj_orient = []
        r_traj_pos = []
        r_traj_orient = []
        time_stamp = []
        object_poses = []
        object_orientations = []
        object_dims = []
        object_names = []

        joint_names = []
        joint_positions = []
        joint_efforts = []

        for _, msg, _ in bag.read_messages(topics="object_and_joints"):
            isinstance(msg, ObjectAndJoints)

            #left_gripper_pose
            pose = msg.left_gripper_pose
            l_traj_pos.append( [pose.pose.position.x,
                                pose.pose.position.y,
                                pose.pose.position.z,
                                ]
                               )
            l_traj_orient.append( [pose.pose.orientation.x,
                                   pose.pose.orientation.y,
                                   pose.pose.orientation.z,
                                   pose.pose.orientation.w,
                                   ]
                                  )

            #right_gripper_pose
            pose = msg.right_gripper_pose
            r_traj_pos.append( [pose.pose.position.x,
                                pose.pose.position.y,
                                pose.pose.position.z,
                                ]
                               )
            r_traj_orient.append( [pose.pose.orientation.x,
                                   pose.pose.orientation.y,
                                   pose.pose.orientation.z,
                                   pose.pose.orientation.w,
                                   ]
                                  )
            time_stamp.append(pose.header.stamp.to_sec())

            #object_poses
            object_poses.append([(pose.pose.position.x,
                                 pose.pose.position.y,
                                 pose.pose.position.z
                                 ) for pose in msg.object_poses
                                 ]
                                )
            object_orientations.append([(pose.pose.orientation.x,
                                         pose.pose.orientation.y,
                                         pose.pose.orientation.z,
                                         pose.pose.orientation.w
                                         ) for pose in msg.object_poses
                                        ]
                                       )

            #box_dims
            object_dims.append([(dims.x,
                                 dims.y,
                                 dims.z
                                 ) for dims in msg.box_dims
                                ]
                               )


            #object_names
            object_names.append(msg.object_names)

            #joints
            joint_names.append(msg.joints.name)
            joint_efforts.append(msg.joints.effort)
            joint_positions.append(msg.joints.position)

        group_name = "trajectory_" + str(traj_num)
        f[group_name + "/time_stamp"] = time_stamp
        f[group_name + "/l_gripper_positions"] = l_traj_pos
        f[group_name + "/l_gripper_orientations"] = l_traj_orient
        f[group_name + "/r_gripper_positions"] = r_traj_pos
        f[group_name + "/r_gripper_orientations"] = r_traj_orient

        f[group_name + "/objects_position"] = object_poses
        f[group_name + "/objects_orientation"] = object_orientations
        f[group_name + "/object_dimension"] = object_dims

        f[group_name + "/joint_names"] = joint_names
        f[group_name + "/joint_efforts"] = joint_efforts
        f[group_name + "/joint_positions"] = joint_positions

        traj_num += 1
    f.close()


if __name__ == "__main__":
    main()
