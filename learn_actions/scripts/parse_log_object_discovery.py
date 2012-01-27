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
    
    
    inpt_mat = []
    oupt_mat = []
    
    print options.input_file.split()
    gen = (glob.glob(f) for f in options.input_file.split())
    for filename in itertools.chain.from_iterable(gen):
        rospy.loginfo("Opening bag %s"% filename)
        bag = rosbag.Bag(filename)
        for _, msg, _ in bag.read_messages(topics="object_discovery"):
            input_vec = (msg.object_pose.x,
                         msg.object_pose.y,
                         msg.object_pose.z,
                         msg.table.x_min,
                         msg.table.x_max,
                         msg.table.y_min,
                         msg.table.y_max,
                        )
            inpt_mat.append(input_vec)

            output_vec = (msg.grasping_result,
                         )
            oupt_mat.append(output_vec)
    
    inpt_mat = numpy.array(inpt_mat)
    oupt_mat = numpy.array(oupt_mat)

    ofile = open(options.output_file, "w")
    cPickle.dump((inpt_mat, oupt_mat), ofile)
    ofile.close()

if __name__ == "__main__":
    main()
