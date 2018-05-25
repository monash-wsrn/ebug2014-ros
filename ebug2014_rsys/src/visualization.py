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
  x_pos = 0.0 # in camera pixel coordinates
  y_pos = 0.0 # in camera pixel coordinates
  angle = 0.0 # in degrees
#--

class arenaGUI(object):
  first_view = True
  data = ''
  num_ebugs = 0
  eBugs = {}
  
  def __init__(self, master):
    self.master = master
    master.title('eBug Arena')
    self.updateArenaView(self.data)

  def updateArenaView(self, data):
    img = np.zeros((512, 640, 3), np.uint8) # Create the blank view
    cv2.putText(img, '%0d' % self.num_ebugs, (0, 50),
		cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 1, cv2.LINE_AA)
    if not self.first_view:
      ebug_poses = data.split(' ') 
      self.num_ebugs = int(ebug_poses[0])
      rospy.loginfo("We see %d eBugs in this view.", self.num_ebugs)
      for i in range(0, self.num_ebugs):
        j = (i * 4) + 1
        ebug_id = int(ebug_poses[j])
        if (ebug_id not in self.eBugs):
          self.eBugs[ebug_id] = eBugPose()
        self.eBugs[ebug_id].x_pos = float(ebug_poses[j+1])
        self.eBugs[ebug_id].y_pos = float(ebug_poses[j+2])
        self.eBugs[ebug_id].angle = float(ebug_poses[j+3])
        c_x = int(self.eBugs[ebug_id].x_pos) // 2
        c_y = int(self.eBugs[ebug_id].y_pos) // 2
        cv2.circle(img, (c_x, c_y), 30, (0,255,0), -1, cv2.LINE_AA)
        cv2.putText(img, '%0d' % ebug_id, (c_x - 10, c_y + 12),
		    cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 255), 3, cv2.LINE_AA)

        theta = self.eBugs[ebug_id].angle
        x1 = c_x + int(30 * math.cos(math.radians(theta)))
        y1 = c_y + int(30 * math.sin(math.radians(theta)))
        cv2.arrowedLine(img, (c_x, c_y), (x1, y1), (0,0,255), 3, 8, 0, 0.5)
      #-- for
    #-- if
    # OpenCV represents images in BGR order, however PIL represents
    # images in RGB order, hence we need to swap the channels:
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(img)    # Convert the image to PIL format, and
    img = ImageTk.PhotoImage(img) # then to ImageTk format. 

    if self.first_view:
      self.arena_view = Label(image=img)
      self.arena_view.pack(side="left", padx=10, pady=10)
      self.first_view = False
    self.arena_view.configure(image=img)
    self.arena_view.image = img
#-- class arenaGUI
		
class getEBugPoses(object):
  data = ''
  
  def __init__(self, av):
    self.sub = rospy.Subscriber("poses", String, self.callback)
    self.av = av
    
  def callback(self, data):
    rospy.loginfo("Callback: %s", data.data)
    self.data = data.data
    self.av.updateArenaView(self.data)
#-- class getEBugPoses

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
