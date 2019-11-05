#!/usr/bin/env python
from ada_demos.srv import DetectAcquisition, DetectAcquisitionResponse
import rospy

def handle_detection(req):
    return DetectAcquisitionResponse(True)

def detector():
    rospy.init_node('acquisition_detector')
    s = rospy.Service('acquisition_detector', DetectAcquisition, handle_detection)
    rospy.spin()

if __name__ == "__main__":
    detector()
