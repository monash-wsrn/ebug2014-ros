from __future__ import absolute_import
import numpy as np
import scipy.sparse.csgraph
from itertools import izip


def Clusters(LEDs, indices, colors, blob_sizes):
    dist_sq = np.array([])
    for neigh in indices:
        dist_sq = np.append(dist_sq, ((LEDs[neigh[0]] - LEDs[neigh[1]]) ** 2).sum())
        dist_sq = np.append(dist_sq, ((LEDs[neigh[0]] - LEDs[neigh[2]]) ** 2).sum())
    mid = np.median(dist_sq)

    LED_graph = np.zeros((len(LEDs), len(LEDs)))
    for i in xrange(len(dist_sq)):
        index = i // 2
        neigh_index = i % 2 + 1
        if dist_sq[i] < mid * 2 and dist_sq[i] > mid * 0.5:
            LED_graph[indices[index][0]][indices[index][neigh_index]] = 1
    labels = scipy.sparse.csgraph.connected_components(LED_graph)
    cluster_nums = [i for i in xrange(labels[0])]
    valid_cluster = [i for i in cluster_nums if labels[1].tolist().count(i) > 5]
    valid_LEDs = np.array([]).reshape(0, 2)
    valid_cluster_index = np.array([])

    valid_color = np.array([])
    valid_blob = np.array([])
    for axis, i, color, blob_size in izip(LEDs, labels[1], colors, blob_sizes):
        if i not in valid_cluster:
            continue
        valid_LEDs = np.vstack((valid_LEDs, axis))
        valid_cluster_index = np.append(valid_cluster_index, i)
        valid_color = np.hstack((valid_color, color))
        valid_blob = np.hstack((valid_blob, blob_size))
    return valid_LEDs, valid_color, valid_blob, valid_cluster_index
