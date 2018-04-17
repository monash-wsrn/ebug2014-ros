#!/usr/bin/env python
from __future__ import division
from __future__ import absolute_import

import rospy
from std_msgs.msg import String
import sys
import numpy as np
import cv2
import math
import time
#import settings
from itertools import izip

def read_poses_cb(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    img = np.zeros((480, 640, 3), np.uint8)
    # Here we update the positions of eBugs on the screen.
    cv2.imshow(u'eBug Arena', img)

def show_arena():
    rospy.init_node('visualization', anonymous=True)
    rospy.Subscriber("poses", String, read_poses_cb)
    cv2.namedWindow(u'eBug Arena')
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    show_arena()


