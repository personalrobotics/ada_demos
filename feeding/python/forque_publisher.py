#!/usr/bin/env python

import rospy
#import cv2
import numpy as np
from geometry_msgs.msg import WrenchStamped
#from cv_bridge import CvBridge

empty_mu = 0.0955551967661348
empty_sigma = 0.1716494429280944

def forque_publisher():
    forque_topic = '/forque/forqueSensor'
    pub = rospy.Publisher(forque_topic, WrenchStamped, queue_size=10)
    rospy.init_node('forquer', anonymous=True)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        forque_msg = WrenchStamped()
        forque_msg.header.seq = 0
        forque_msg.header.stamp = rospy.Time.now()
        forque_msg.header.frame_id = 'forqueSensorFrame'
        forque_msg.wrench.force.x = 0.00
        forque_msg.wrench.force.y = 0.00
        forque_msg.wrench.force.z = np.random.normal(empty_mu, empty_sigma)
        forque_msg.wrench.torque.x = 0.00
        forque_msg.wrench.torque.y = 0.00
        forque_msg.wrench.torque.z = 0.00
        #print(forque_msg)
        pub.publish(forque_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        forque_publisher()
    except rospy.ROSInterruptException:
        pass

