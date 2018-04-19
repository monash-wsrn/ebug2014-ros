#!/usr/bin/env python
from __future__ import division

import rospy, sys, cv2, math, time, numpy as np
from std_msgs.msg import String
from itertools import izip
#import settings

'''
We get the eBug pose information in the callback, and use it in the main
to update the eBug Arena view. 
Directly dealing with the imagery in the callback does not work. 
See: https://answers.ros.org/question/257440/python-opencv-namedwindow-and-imshow-freeze/
'''
class getEBugPoses():
    def __init__(self):
        self.sub = rospy.Subscriber("poses", String, self.callback)
        self.params = None # Do we need it?

    def callback(self, data):
#       rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        rospy.loginfo("Callback: %s", data.data)
        self.data = data.data
# end class updateArenaView()

def show_arena():
    rospy.init_node('visualization', anonymous=True)
    eBugPoses = getEBugPoses()
    cv2.namedWindow(u'eBug Arena')
    arena_bg_img = cv2.imread('arena_bg.png')

    count = 0
    while (True):
        img = np.zeros((480, 640, 3), np.uint8)
        rospy.loginfo("Main: %s", eBugPoses.data)
        # Place the eBugs on arena view by using the eBugPoses.data
        cv2.circle(img, (100, 200), 50, (0,255,0), -1)
        cv2.putText(img, u'%0d' % count, (0, 50), \
                    cv2.FONT_HERSHEY_PLAIN, 4, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.imshow(u'eBug Arena', img)
        k = cv2.waitKey(1) & 0xFF
        if k == 27: break
        count += 1
    
    rospy.spin() # Simply keeps python from exiting until this node is stopped.
    # cv2.destroyAllWindows()
    
if __name__ == '__main__':
    try:
        show_arena()
    except rospy.ROSInterruptException:
        pass
