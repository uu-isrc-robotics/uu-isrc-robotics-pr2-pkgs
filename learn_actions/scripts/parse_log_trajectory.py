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
from learn_actions.msg import ObjectTrajectoryResults
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

        for _, msg, _ in bag.read_messages(topics="/trajectory_effect"):
            isinstance(msg, ObjectTrajectoryResults)

            traj_pos = []
            traj_orient = []
            time_stamp = []
            for pose in msg.trajectory:
                isinstance(pose, PoseStamped)
                traj_pos.append( [pose.pose.position.x,
                                  pose.pose.position.y,
                                  pose.pose.position.z,
                                  ]
                                 )
                traj_orient.append( [pose.pose.orientation.x,
                                     pose.pose.orientation.y,
                                     pose.pose.orientation.z,
                                     pose.pose.orientation.w,
                                     ]
                                    )
                time_stamp.append(pose.header.stamp.to_sec())

            gripper_opening = msg.gripper_open
            object_pose = [(pose.pose.position.x,
                           pose.pose.position.y,
                           pose.pose.position.z
                           ) for pose in msg.pre_movement_object_poses
                          ]
            object_orientation = [(pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z,
                                  pose.pose.orientation.w
                                  ) for pose in msg.pre_movement_object_poses
                                 ]
            object_dims = [(dims.x,
                           dims.y,
                           dims.z
                           ) for dims in msg.pre_movement_box_dims
                          ]

            group_name = "trajectory_" + str(traj_num)
            f[group_name + "/time_stamp"] = time_stamp
            f[group_name + "/gripper_positions"] = traj_pos
            f[group_name + "/gripper_orientations"] = traj_orient
            f[group_name + "/objects_position"] = object_pose
            f[group_name + "/objects_orientation"] = object_orientation
            f[group_name + "/object_dimension"] = object_dims
            f[group_name + "/gripper_open"] = gripper_opening

            traj_num += 1
    f.close()


if __name__ == "__main__":
    main()
