#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def show_image():
    rospy.init_node('visualization', anonymous=True)
# we get the blob information and show
# camera.py has the statements.
# !!! NOT FINISHED

if __name__ == '__main__':
    try:
        show_image()
    except rospy.ROSInterruptException:
        pass

