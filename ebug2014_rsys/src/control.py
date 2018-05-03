#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class ebug_poses(object):
    def __init__(self):
        self.fp = open('1ebug_85cm_moving_frames.log', 'r')

    def __call__(self):
        return self.fp.readline()
# end class ebug_poses()

def send_commands():
    rospy.loginfo('Sending eBug control commands: Not implemented yet.')
# end control()

if __name__ == '__main__':
    sense = ebug_poses()
    try:
        while not rospy.is_shutdown():
            pub = rospy.Publisher('blobs', String, queue_size=100)
            rospy.init_node('control', anonymous=True)
            rate = rospy.Rate(30) # 30hz

            frame = sense() # get a frame from the blob camera
            rospy.loginfo(frame)

            send_commands() # send control commands
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
