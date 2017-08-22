#!/usr/bin/python
from __future__ import division
from __future__ import absolute_import
from libraries import nrf
from math import pi, sqrt
import numpy as np
from sklearn.neighbors import NearestNeighbors
from libraries.clustering import Clusters
import libraries.circle as CL
# from time import time
import json
from itertools import izip
from itertools import imap

LED_DETECTION_THRESHOLD = 6


def contact_camera():
    nrf_bridge = nrf.Bridge(u'/dev/ttyACM0')  # or '/dev/ttyACM0'
    a = nrf_bridge.assign_addresses()

    for i in list(a.keys()):
        nrf_bridge.set_TX_address(i)
        if nrf_bridge.get_ID_type()[6] == 1:  # find first camera in neighbours
            break
    else:
        raise RuntimeError(u'No camera found')
    return nrf_bridge


def get_robot_position(ts, camera):
    try:
        ts, blobs_list = camera.get_blobs()
        prev_ts = ts
        while ts == prev_ts:
            ts, blob = camera.get_blobs()
            blobs_list += blob
        LED_positions = []
        LED_position_buffer = []
        LED_colors_buffer = []
        blob_size_buffer = []
        frame_times = list(izip(xrange(10), xrange(10)))
        if ts != prev_ts:
            for blob in blobs_list:
                # A blob contains one lED position
                x_coord = blob[0]
                y_coord = blob[1]
                color = blob[2]
                radius = blob[3]
                LED_position_buffer.append([x_coord / 2, y_coord / 2])
                LED_colors_buffer.append(color)
                blob_size_buffer.append(int(radius / sqrt(pi) / 2) * 2)
                LED_positions = np.array(LED_position_buffer)
                LED_colors = np.array(LED_colors_buffer)
                blob_sizes = np.array(blob_size_buffer)
            if len(LED_positions) > 5:
                # print('Captage de ', len(LED_positions), ' LEDs')
                # Find the closest neighbors of each LED_positions
                neigh = NearestNeighbors(n_neighbors=4, algorithm=u'ball_tree').fit(LED_positions)
                distances, indices = neigh.kneighbors(LED_positions)
                # print(distances, indices)
                index_true = []
                index_false = []
                for i in xrange(len(distances)):
                    if indices[i][0] in index_false:
                        continue
                    bad_group = []
                    for j in xrange(1, 4):
                        if distances[i][j] < 6:
                            bad_group.append(indices[i][j])
                    index_true += [indices[i][0]]
                    index_false += bad_group

                LED_positions = LED_positions[list(set(index_true))]
                if len(LED_positions) > 4:
                    LED_colors = LED_colors[list(set(index_true))]
                    blob_sizes = blob_sizes[list(set(index_true))]
                    nbrs = NearestNeighbors(n_neighbors=3, algorithm=u'kd_tree').fit(LED_positions)
                    indices = nbrs.kneighbors(LED_positions, return_distance=False)
                    # Group LEDs in order to create ebugs
                    valid_LEDs, LED_colors, blob_sizes, valid_cluster_index = Clusters(LED_positions, indices,
                                                                                       LED_colors,
                                                                                       blob_sizes)
                    centers = []
                    LEDs_per_Ebug = []
                    LEDColor_per_Ebug = []
                    BlobSize_per_Ebug = []
                    radius = []
                    visited = []
                    ID = []
                    for i in valid_cluster_index:
                        if i in visited:
                            continue
                        visited.append(i)
                        buf_LEDs = valid_LEDs[np.where(valid_cluster_index == i)]
                        circle_LEDs = CL.circle(buf_LEDs)
                        center = np.array(list(imap(int, circle_LEDs.LS())))
                        centers.append(center)
                        LEDs_per_Ebug.append(buf_LEDs)
                        radius.append(int(circle_LEDs.calc_R(*center).mean()))
                        LEDColor_per_Ebug.append(LED_colors[np.where(valid_cluster_index == i)])
                        BlobSize_per_Ebug.append(blob_sizes[np.where(valid_cluster_index == i)])
                    for i in xrange(len(radius)):
                        color_seq, blob_seq = CL.GetSequence(LEDs_per_Ebug[i], LEDColor_per_Ebug[i],
                                                             BlobSize_per_Ebug[i],
                                                             centers[i], radius[i], 16)
                        EbugID = CL.EbugIdDtection(color_seq, LED_DETECTION_THRESHOLD)
                        if EbugID != -1:
                            ID.append(EbugID)
                    ebugs_list = []
                    for i in xrange(len(ID)):
                        ebug = {
                            u"ts": int(prev_ts),
                            u"id": int(ID[i]),
                            u"x": int(centers[i][0]),
                            u"y": int(centers[i][1]),
                            u"Radius": int(radius[i]),
                            u"angle": 0
                        }
                        ebugs_list.append(ebug)
                    if len(ebugs_list) != 0:
                        str_array = []
                        for ebug in ebugs_list:
                            str_array.append(dict(ebug))
                        return json.dumps(str_array)
                    return None
        return None
    except RuntimeError, e:
        pass
        # old_time = frame_times[0]
        # new_time = (ts, time())
        # frame_times = frame_times[1:] + [new_time]
        # fps_camera = 1000. * len(frame_times) / (new_time[0] - old_time[0])
        # fps_local = len(frame_times) / (new_time[1] - old_time[1])


def main():
    camera = contact_camera()
    i = 0
    # start = time()
    while True:
        get_robot_position(0, camera)


if __name__ == u'__main__':
    main()
