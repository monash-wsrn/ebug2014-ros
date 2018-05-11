#!/usr/bin/env python
from __future__ import division
from __future__ import absolute_import
from __future__ import print_function

import rospy, sys, cv2, math, time, numpy as np
from std_msgs.msg import String
from itertools import izip
from Tkinter import *
from PIL import Image, ImageTk # Python Imaging Library
#import settings

class eBugPose:
  x_pos = 0.0
  y_pos = 0.0
  angle = 0.0
#

class arenaGUI(object):
  first_view = True
  data = ''
  num_ebugs = 0
  eBugs = {}
  
  def __init__(self, master):
    self.master = master
    master.title('eBug Arena')
    self.updateArenaView(self.data)
    eBugs = [eBugPose() for i in range(16)] # We can support max 16 eBugs currn.

  def updateArenaView(self, data):
    self.img = np.zeros((512, 640, 3), np.uint8) # Create the blank view
    cv2.putText(self.img, '%0d' % self.num_ebugs, (0, 50),
		cv2.FONT_HERSHEY_PLAIN, 4, (0, 0, 255), 1, cv2.LINE_AA)
    self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
    self.img = Image.fromarray(self.img)
    self.img = ImageTk.PhotoImage(self.img)
    
    if self.first_view:
      self.arena_view = Label(image=self.img)
      self.arena_view.image = self.img
      self.arena_view.pack(side="left", padx=10, pady=10)
      self.first_view = False
    else:
      self.num_ebugs, self.ebug_poses = data.split(' ', 1)
      self.num_ebugs = int(self.num_ebugs)
      for i in range(1, self.num_ebugs):
        print('tomorrow complete here')
#        eBugs[int(ebug_poses[i])]. 
      self.arena_view.configure(image=self.img)
      self.arena_view.image = self.img
# <=> class arenaGUI
		
class getEBugPoses(object):
  data = ''
  
  def __init__(self, av):
    self.sub = rospy.Subscriber("poses", String, self.callback)
    self.av = av
    
  def callback(self, data):
    rospy.loginfo("Callback: %s", data.data)
    self.data = data.data
    self.av.updateArenaView(self.data)
# <=> class getEBugPoses

if __name__ == '__main__':
  try:
    rospy.init_node('visualization', anonymous=True)
    root = Tk()
    arenaView = arenaGUI(root)
    eBugPoses = getEBugPoses(arenaView)
    root.mainloop()
    # rospy.spin()
  except rospy.ROSInterruptException:
    pass
  
  
"""
If there is another blocking call that keeps your program running,
there is no need to call rospy.spin(). Unlike in C++ where spin() is
needed to process all the threads, in python all it does is block.
"""
