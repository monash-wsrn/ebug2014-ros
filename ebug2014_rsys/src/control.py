#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class ebug_poses(object):
    def __init__(self):
        self.fp = open('ebugs_03_08_stationary_20180426.log', 'r')

    def __call__(self):
        return self.fp.readline()
# <--class ebug_poses()

# ToDo This will turn into a class I think
def send_commands():
    rospy.loginfo('Sending eBug control commands: Not implemented yet.')
# <--send_commands()

if __name__ == '__main__':
    try:
		sense = ebug_poses()
		pub = rospy.Publisher('blobs', String, queue_size=100)
		rospy.init_node('control', anonymous=True)
		rate = rospy.Rate(30) # 30 Hz
		while not rospy.is_shutdown():
			frame = sense() # get a frame from the blob camera
			rospy.loginfo(frame)
			pub.publish(frame)
			send_commands() # send control commands
			rate.sleep()
    except rospy.ROSInterruptException:
        pass


	
