#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_publisher():
    br = CvBridge()
    image_topic = '/camera/color/image_raw/'
    pub = rospy.Publisher(image_topic, Image, queue_size=10)
    rospy.init_node('imager', anonymous=True)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        #image_path = './empty_fork.png' # Always empty fork
        #image_path = './food_on_fork.png' # Always food on fork
        image_path = './food_on_fork.png' if np.random.randint(0, 2) == 1 else './empty_fork.png'
        cv_image = cv2.imread(image_path)
        image_msg = br.cv2_to_imgmsg(cv_image, "bgr8")
        pub.publish(image_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass

