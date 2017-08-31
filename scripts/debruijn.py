#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 31 16:00:34 2017

@author: asekerci
"""
debruijn_sequences = ('BBGRRBRBGBBGBBRG',
                      'RGRRRGRBGBRGBGRG',
                      'BRGGGGRRRBBBBBGR',
                      'RBRGRBBRBBBRGRGB',
                      'GGBGBRBBGRGBBBGG',
                      'RGBRGGBBBRRGBGGR',
                      'GRRBBRGGRBBBGBRR',
                      'BGGRBRBBRRRBGBGR',
                      'GBRRBBGGBRBGRRGG',
                      'RGGRGBGBBBBRBGGG',
                      'RGBRRRRRGBBRBRRB',
                      'RRGRGGGRBGRGGBRG',
                      'RGRBRRRGGBGGBBGG',
                      'GGGBBRRBGRBBGBGB',
                      'RBGGBGRRRRBRRGGR')

leds = (0x0800, 0x1000, 0x2000,	0x4000, 0x8000, 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,	0x0100, 0x0200, 0x0400)

for debruijn_sequence in debruijn_sequences:
    red = 0; green = 0; blue = 0
    for i, letter in enumerate(debruijn_sequence):
        if letter == 'R': red += leds[i]
        if letter == 'G': green += leds[i]
        if letter == 'B': blue += leds[i]
    print debruijn_sequence, hex(red), hex(green), hex(blue)
# for        