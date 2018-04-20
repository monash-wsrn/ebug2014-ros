#!/usr/bin/python

from __future__ import division
from __future__ import absolute_import

import sys
import numpy as np
import cv2
import math
import time
#import settings
from itertools import izip

sys.path.insert(0, '../libs/') 

from nrf import Bridge

class thresholds(object):
    values = [0xA2, 0x99, 0x76, 0x85, 0xB1, 0x8E, 0xa3, 0x82]

    def __init__(self, i):
        self.i = i

    def __call__(self, value):
        old = thresholds.values[self.i]
        thresholds.values[self.i] = value
        try:
            nrf.set_camera_thresholds(thresholds.values)
        except RuntimeError, e:  # ACK not received, set trackbar back
                                 # to original value
            thresholds.values[self.i] = old
            cv2.setTrackbarPos([u'Red', u'Green', u'Blue', u'Magenta'] \
                               [self.i / 2] + u' ' + [u'U', u'V'] \
                               [self.i % 2], u'Blob Camera', old)
        # 
    # 
# end class thresholds


nrf = Bridge(u'/dev/ttyACM0')  # or '/dev/ttyACM1' ToDo 
a = nrf.assign_addresses()

for i in list(a.keys()):
    nrf.set_TX_address(i)
    if nrf.get_ID_type()[6] == 1:  # find first camera in neighbours
        camera_address = i
        break
else:
    raise RuntimeError(u'No cameras found')

# We are operating the camera at SXGA resolution.
# overclock PSoC and camera (default is 48MHz, which gives 15fps SXGA,
# 30fps VGA, 60fps CIF and 120fps QCIF)
# nrf.send_packet('\x94'+chr(64)) 

cv2.namedWindow(u'Blob Camera')
i = 0
for colour in [u'Red', u'Green', u'Blue', u'Magenta']:
    for channel in [u'U', u'V']:
        cv2.createTrackbar(colour + u' ' + channel, u'Blob Camera', \
                           thresholds.values[i], 255, thresholds(i))
        i += 1
    # end for
# end for
cv2.createTrackbar(u'Exposure',u'Blob Camera',40,1024, \
                   lambda x:nrf.set_camera_exposure(x)) # exposure in image
                                                        # rows (max is height
                                                        # of image)
cv2.createTrackbar(u'Analogue gain',u'Blob Camera',0,255,
                   lambda x:nrf.set_camera_gain(x)) # analogue gain
                                                    # (best to keep at 0
                                                    # and adjust exposure)
cv2.createTrackbar(u'Blue gain',u'Blob Camera',128,255, \
                   lambda x:nrf.set_camera_blue_gain(x)) # white balance:
                                                         # gain adjustment for
                                                         # blue channel
cv2.createTrackbar(u'Red gain',u'Blob Camera',128,255, \
                   lambda x:nrf.set_camera_red_gain(x)) # white balance:
                                                        # gain adjustment for
                                                        # red channel
nrf.camera_write_reg(0x13,0) # disable AEC, AGC, and AWB

ts = 0
frame_blobs = []
colours = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 0, 255)]
frame_times = list(izip(xrange(10), xrange(10)))
while True:
    try:
        k = cv2.waitKey(1)
        if unichr(k & 0xff) == u'q': break
        prev_ts = ts
        ts, blobs = nrf.get_blobs()
        if ts != prev_ts:
            img = np.zeros((512, 640, 3), np.uint8)
            for b in frame_blobs:
                cv2.circle(img, (b[0] // 2, b[1] // 2), \
                           int(b[3] / math.pi ** 0.5 / 2), \
                           colours[b[2]], -1, cv2.LINE_AA)
            old_time = frame_times[0]
            new_time = (ts, time.time())
            frame_times = frame_times[1:] + [new_time]
            fps_camera = 1000. * len(frame_times) / (new_time[0] - old_time[0])
            fps_local = len(frame_times) / (new_time[1] - old_time[1])
            cv2.putText(img, u'%03.0f' % fps_camera, (0, 20), \
                        cv2.FONT_HERSHEY_PLAIN, 1, (128, 128, 0), 1, \
                        cv2.LINE_AA)
            cv2.putText(img, u'%03.0f' % fps_local, (40, 20), \
                        cv2.FONT_HERSHEY_PLAIN, 1, (128, 128, 0), 1, \
                        cv2.LINE_AA)
            cv2.imshow(u'Blob Camera', img)
            frame_done = True
            print(ts, frame_blobs)
            frame_blobs = []
        # end if
        frame_blobs += blobs
        
    except RuntimeError, e:
        print e
        continue  # TODO check what kind of error
    # end try
# end while

