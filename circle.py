from __future__ import division
from __future__ import absolute_import
import numpy as np
from scipy import optimize
import math


class circle(object):
    def __init__(self, num):
        self.LEDs = num
        self.x = np.choose([0] * len(num), num.T).astype(float)
        self.y = np.choose([1] * len(num), num.T).astype(float)

    def calc_R(self, xc, yc):
        return np.sqrt((self.x - xc) ** 2 + (self.y - yc) ** 2)

    def f_2(self, c):
        Ri = self.calc_R(*c)
        return Ri - Ri.mean()

    def LS(self):
        center_estimate = np.mean(self.x), np.mean(self.y)
        center_2, ier = optimize.leastsq(self.f_2, center_estimate)
        return center_2


def GetSequence(led, color, blob_size, center, R, N):
    R_2 = R ** 2
    color_seq = [4] * N
    blob_seq = [0.0] * N
    led.astype(float)

    for i, point in enumerate(led):
        if ((point - center) ** 2).sum() < R_2 * 0.8 or ((point - center) ** 2).sum() > R_2 * 1.2:
            continue
        r_y, r_x = point[1] - center[1], point[0] - center[0]
        if r_y >= 0:
            index = int(math.floor(math.atan2(r_y, r_x) / math.pi * N / 2))
            if color_seq[index] == 4:
                color_seq[index] = color[i]
                blob_seq[index] = blob_size[i]
            else:
                if blob_seq[index] > blob_size[i]:
                    continue
                color_seq[index] = color[i]
                blob_seq[index] = blob_size[i]
        if r_y < 0:
            index = int(math.floor(math.atan2(r_y, r_x) / math.pi * N / 2 + N))
            if color_seq[index] == 4:
                color_seq[index] = color[i]
                blob_seq[index] = blob_size[i]
            else:
                if blob_seq[index] > blob_size[i]:
                    continue
                color_seq[index] = color[i]
                blob_seq[index] = blob_size[i]
    return color_seq, blob_seq


def strstr(source, target):
    if target == None or len(target) == 0:
        return -1
    for i in xrange(len(source) - len(target) + 1):
        if source[i:i + len(target)] == target:
            return 1
    return -1


def EbugIdDtection(color_seq, thres):
    led_dict = [[2, 2, 1, 0, 0, 2, 0, 2, 1, 2, 2, 1, 2, 2, 0, 1, 2, 2, 1, 0, 0, 2, 0, 2, 1, 2, 2, 1, 2, 2, 0],
                [0, 1, 0, 0, 0, 1, 0, 2, 1, 2, 0, 1, 2, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 2, 1, 2, 0, 1, 2, 1, 0],
                [2, 0, 1, 1, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 1, 0, 2, 0, 1, 1, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 1],
                [0, 2, 0, 1, 0, 2, 2, 0, 2, 2, 2, 0, 1, 0, 1, 2, 0, 2, 0, 1, 0, 2, 2, 0, 2, 2, 2, 0, 1, 0, 1],
                [1, 1, 2, 1, 2, 0, 2, 2, 1, 0, 1, 2, 2, 2, 1, 1, 1, 1, 2, 1, 2, 0, 2, 2, 1, 0, 1, 2, 2, 2, 1],
                # [0,1,2,0,1,1,2,2,2,0,0,1,2,1,1,0,0,1,2,0,1,1,2,2,2,0,0,1,2,1,1],
                [1, 0, 0, 2, 2, 0, 1, 1, 0, 2, 2, 2, 1, 2, 0, 0, 1, 0, 0, 2, 2, 0, 1, 1, 0, 2, 2, 2, 1, 2, 0],
                # [2,1,1,0,2,0,2,2,0,0,0,2,1,2,1,0,2,1,1,0,2,0,2,2,0,0,0,2,1,2,1],
                # [1,2,0,0,2,2,1,1,2,0,2,1,0,0,1,1,1,2,0,0,2,2,1,1,2,0,2,1,0,0,1],
                [0, 1, 1, 0, 1, 2, 1, 2, 2, 2, 2, 0, 2, 1, 1, 1, 0, 1, 1, 0, 1, 2, 1, 2, 2, 2, 2, 0, 2, 1, 1],
                # [0,1,2,0,0,0,0,0,1,2,2,0,2,0,0,2,0,1,2,0,0,0,0,0,1,2,2,0,2,0,0],
                # [0,0,1,0,1,1,1,0,2,1,0,1,1,2,0,1,0,0,1,0,1,1,1,0,2,1,0,1,1,2,0],
                # [0,1,0,2,0,0,0,1,1,2,1,1,2,2,1,1,0,1,0,2,0,0,0,1,1,2,1,1,2,2,1],
                # [1,1,1,2,2,0,0,2,1,0,2,2,1,2,1,2,1,1,1,2,2,0,0,2,1,0,2,2,1,2,1],
                [0, 2, 1, 1, 2, 1, 0, 0, 0, 0, 2, 0, 0, 1, 1, 0, 0, 2, 1, 1, 2, 1, 0, 0, 0, 0, 2, 0, 0, 1, 1]]
    vote = [0] * len(led_dict)
    for i in xrange(len(color_seq) - thres + 1):
        for j in xrange(len(led_dict)):
            cnt = strstr(led_dict[j], color_seq[i:i + thres])
            if cnt == 1:
                vote[j] += 1
                break

    buf = [(k, j) for k, j in enumerate(vote) if j > 0]
    # if len(buf) == 1:
    #  return buf[0]
    if max(vote) > 0:
        return vote.index(max(vote))
    else:
        return -1
