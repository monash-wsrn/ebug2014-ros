#!/usr/bin/python
from __future__ import absolute_import
import settings
from libraries.nrf import Bridge
import sys

if u'all' in sys.argv:
    flash_all = True
    sys.argv.remove(u'all')
else:
    flash_all = False

if len(sys.argv) < 2:
    raise RuntimeError(u'usage: %s master|slave|<filename> [all] [<serial port>]' % sys.argv[0])

if len(sys.argv) < 3:
    nrf = Bridge()
else:
    nrf = Bridge(sys.argv[2])

master = u'../ebug2014-firmware/eBug2014 Master.cydsn/CortexM3/ARM_GCC_493/Release/eBug2014 Master.cyacd'
slave = u'../ebug2014-firmware/eBug2014 Slave.cydsn/CortexM3/ARM_GCC_493/Release/eBug2014 Slave.cyacd'

flash_func = nrf.flash_all_ebugs if flash_all else nrf.flash

if sys.argv[1] == u'master':
    flash_func(master)
elif sys.argv[1] == u'slave':
    flash_func(slave)
else:
    flash_func(sys.argv[1])
