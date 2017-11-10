#!/usr/bin/python

import sys

sys.path.insert(0, '../libs/') 

from nrf import Bridge

if len(sys.argv) == 2:
    if sys.argv[1] == '-c':
        camera_calibration = True
        print 'Entered camera calibration mode.'
    else:
        # -c: for turning on the calibration LED patterns
        print 'Usage:', sys.argv[0], '[-c]' 
        sys.exit(1)
else:
    camera_calibration = False

    
nrf = Bridge()
cameras, ebugs, unknowns = nrf.assign_static_addresses('../libs/ebug_tab.json')

for addr, info in ebugs.items():
    print 'Initializing eBug: ', addr
    nrf.set_TX_address(addr)
    nrf.enable_LEDs(0, 1, 0, 0) 
    nrf.LED_brightness(8)
    nrf.LCD_backlight(0)
    if camera_calibration == True:
        # Calibration LED pattern (upward looking set):
        # D2 is illuminated red - front (0 degrees) - GGRGG
        # D18 is illuminated blue - (180 degrees) - GGBGG
        red   = 0x0800 + 0x4000 + 0x8000 + 0x0001 + 0x0040 + 0x0080 + 0x0100 
        green = 0x1000 + 0x2000 + 0x0002 + 0x0020 + 0x0200 + 0x0400
        blue  = 0x0010 + 0x0008 + 0x0004 
        nrf.set_LEDs(red, green, blue)
    else:
        red = info['led_sequence'][0]
        green = info['led_sequence'][1]
        blue = info['led_sequence'][2]
        nrf.set_LEDs(int(red, 16), int(green, 16), int(blue, 16))
print 'Initialized', len(ebugs), 'eBug(s)'






