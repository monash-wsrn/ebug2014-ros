from __future__ import with_statement
from __future__ import division
from __future__ import absolute_import
#import settings
import time
import sys
import threading
from multiprocessing import Process
from io import open

sys.path.insert(0, '../libs/') 

from nrf import Bridge

nrf = Bridge()

while True:
    camera, eBugs, unknown = nrf.assign_static_addresses(path = \
                                                     u'../libs/ebug_tab.json')
    if camera:
        break
    else:
        print u" -------- Waiting for camera --------"
        time.sleep(0.03)

#set communication with the first camera detected
#to be changed when multiple cameras
nrf.set_TX_address(iter(camera).next())

#settings camera
# Do not play with the settings, we will fix them with camera.py
# So commenting them out. 
# nrf.camera_write_reg(0x10, 20)
# values = [0x79, 0x9a, 0xb1, 0x5a, 0xc6, 0x70, 0xa3, 0x82]
# nrf.set_camera_thresholds(values)

def stat(maxINT):
    u'''
    @param int maxINT : the number of get_blobs() calls
    @return : displays various stats on the function get_blobs()
    '''
    array = list()
    mean = 0
    fail_number = 0
    for i in xrange(maxINT):
        sys.stdout.write(u'\r%d%%' % (i/maxINT*100))
        sys.stdout.flush()
        try:
            start = time.time()
            nrf.get_blobs()
            end = time.time()
            array.append((end-start)*1000)
        except RuntimeError:
            fail_number += 1
            continue

    mini = array[0]
    maxi = array[0]
    for value in array:
        mean += value
        if value > maxi:
            maxi = value
        elif value < mini:
            mini = value
    mean /= len(array) 

    print u"\nMini : %f ms, Maxi : %f ms, Mean : %f ms, Fail number : %d, \
            Fail rate : %f %%\n" % (mini, maxi, mean, fail_number, \
                                         fail_number/maxINT*100)
# end stat()

def record(seconds, file):
    u'''
    @param int seconds, string file
    record full blobs obtained during @seconds in the @file specified
    '''
    start = time.time()
    end = time.time()

    with open(file,u'w+') as saving_file:
        while(end - start < seconds):
            try:
                blob = nrf.get_blobs()
                saving_file.write(unicode(blob)+u'\n')
            except:
                saving_file.write(u'--> Blob error\n')
            end = time.time()
# end record()

# Tests with timer and multiprocessing     
def get_single_frame(first_blob = None):
    timestamp = set()
    frame = []
    last_blob = None

    while True:
        try:
            blob = nrf.get_blobs()
        except RuntimeError:
            continue
        print blob
        timestamp.add(blob[0])
        if not blob[1]:
            break
        if len(timestamp) > 1:
            last_blob = blob
            break

        frame.append(blob[1])

    timestamp = sorted(timestamp)[0]
    if first_blob and first_blob[0] == timestamp:
        frame.append(first_blob[1])
    return (timestamp, frame), last_blob
# end get_single_frame()

def write_in_file(file, frame):
    with open(file,u'a+') as saving_file: 
        saving_file.write(unicode(frame) + u'\n')
# end write_in_file()

def blob_timer():
    time.sleep(1)
# end blob_timer()

def frame_treatment(frame):
    if frame[1]:
        #write_in_file('test.txt',time.time())
        write_in_file(u'test.txt', frame[0])
# end frame_treatment()

def get_frames():
    next_blob = None
    while(True):
        try:
            timer = Process(target = blob_timer)
            timer.start()
            frame, next_blob = get_single_frame(next_blob)
            Process(target = frame_treatment, args=(frame,)).start()
            timer.join()
            if next_blob:
                print u'frame was full'
        except KeyboardInterrupt:
            break
# End test with timer and multiprocessing

FRAMES = list()

def log_blobs(MAX_TIME, log_file_name):
    u'''
    @params int MAX_TIME, string log_file
    write in the @log_file file located in the folder /scripts/logs 
    information about blobs
    warning : the logs folder must exist  
    '''
    with open(u'logs/' + log_file_name, u'w+') as log_file:
        frame = []
        i = 0
        # discard buffer by calling get_blobs() for a few seconds
        # the first blob recorded in the file can correspond to the
        # middle of a frame, ie first frame may be uncomplete
        while i < 3000:
            try:
                blob = nrf.get_blobs()
            except RuntimeError:
                pass
            i += 1

        frame_number = 1

        print u'Started logging.'

        while True:
            time_0 = time.time()
            try:
                blob = nrf.get_blobs()
            except RuntimeError:
                continue
            time_1 = time.time()
            break

        frame.extend(blob[1])
        RTT = time_1 - time_0 # depends of the size of the blob
        camera_time_0 = camera_time = blob[0]

        log_file.write(u"unix: %f ms, RTT: %f, cam_tstamp : %d ms, blob_info : %s\n" % ((time_1 - time_0) * 1000, RTT, camera_time - camera_time_0, unicode(blob[1])))

        while time_1 - time_0 < MAX_TIME:
            try:
                time_1 = time.time()
                try:
                    blob = nrf.get_blobs()
                except RuntimeError, error:
                    ex_type, ex, tb = sys.exc_info()
                    traceback.print_tb(tb)
                    log_file.write(u"RunTime Error : %s\n" % error)
                    continue
                if camera_time != blob[0]:
                    log_file.write(u"frame# %d %d %s\n" % \
                                    (frame_number, camera_time, unicode(frame)))
                    frame_number += 1
                    if frame:
                        FRAMES.append(frame)
                    frame = []
                if blob[1]:
                    frame.extend(blob[1])
                camera_time = blob[0]
                time_2 = time.time()
                RTT = time_2 - time_1 # depends on the size of the blob
                log_file.write(u"unix: %f ms, RTT: %f, cam_tstamp (rel) : %d ms, cam_tstamp : %d ms, blob_info : %s\n" % ((time_2 - time_0) * 1000, RTT, camera_time - camera_time_0, camera_time, unicode(blob[1])))
            except KeyboardInterrupt:
                break
            # end try
        # end while
    print 'Finished logging.'     
    # end with
# end log_blobs()

