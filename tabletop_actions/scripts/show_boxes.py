#! /usr/bin/python
PKG="tabletop_actions"

import roslib
roslib.load_manifest(PKG)
import rospy
from tabletop_actions.object_detector import GenericDetector
from tabletop_object_detector.srv import TabletopDetectionResponse
import random

def main():
    rospy.init_node("show_boxes", anonymous=True)
    detector = GenericDetector(tabletop_segmentation="find_table")
    res = detector.segment_only().detection
    if res is None:
        rospy.logerr("No object found!")
        return

    rospy.loginfo("I've found %d boxes", len(res.clusters))
    for i, cluster in enumerate(res.clusters):
        box = detector.detect_bounding_box(cluster)
        if box is None:
            rospy.logerr("Error while plotting a box")
            continue
        r = random.random()
        g = random.random()
        b = random.random()

        detector.draw_bounding_box(i, box, (0.7, r,g,b))

    rospy.loginfo("Done")

if __name__ == "__main__":
    main()