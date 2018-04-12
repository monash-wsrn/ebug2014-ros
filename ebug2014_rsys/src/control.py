#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def sense_control():
    pub = rospy.Publisher('blobs', String, queue_size=100)
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 1
    while not rospy.is_shutdown():
        # This is a line from the logfile: 1ebug_85cm_moving_frames.log
        # The blob data  will come from the camera. (coming soon) 
        frame = str(count) + '2 1822945022  762  258  1  4    737  256  0  3    717  262  1  4    780  267  2  5    698  276  2  5    796  285  2  4    687  296  2  5    803  306  2  5    680  318  0  1    688  321  0  2    804  330  1  4    693  340  2  5    791  350  1  4    709  361  1  4    728  368  2  6    777  366  1  4    753  373  1  4' 
        rospy.loginfo(frame)
        pub.publish(frame)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        sense_control()
    except rospy.ROSInterruptException:
        pass

