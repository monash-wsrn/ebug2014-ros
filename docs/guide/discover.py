#!/usr/bin/python

from nrf import Bridge
import sys

sys.path.insert(0, '../libs/')



if len(sys.argv) == 2:
    if sys.argv[1] == '-c':
        camera_calibration = True
        print('Entered camera calibration mode.')
    else:
        print('Usage:', sys.argv[0], '[-c]')  # for turning on the calibration LED patterns
        sys.exit(1)
else:
    camera_calibration = False


nrf = Bridge()
cameras, ebugs, unknowns = nrf.assign_static_addresses('../libs/ebug_tab.json')

for addr, info in list(ebugs.items()):
    print('Initializing eBug: ', addr)
    nrf.set_TX_address(addr)
    nrf.enable_LEDs(0, 1, 0, 0)
    nrf.LED_brightness(4)
    nrf.LCD_backlight(0)
    if camera_calibration is True:
        # Calibration LED pattern (upward looking set):
        # D2 is illuminated green (0 degrees)
        red = 0x0000 + 0x1000 + 0x2000 + 0x4000 + 0x8000 + 0x0001  # LEDs  D4 to D10
        green = 0x0800 + 0x0002 + 0x0004 + 0x0008 + 0x0010 + 0x0020  # LEDs D12 to D22
        blue = 0x0000 + 0x0040 + 0x0080 + 0x0100 + 0x0200 + 0x0400  # LEDs D24 to D32
        nrf.set_LEDs(red, green, blue)
    else:
        red = info['led_sequence'][0]
        green = info['led_sequence'][1]
        blue = info['led_sequence'][2]
        nrf.set_LEDs(int(red, 16), int(green, 16), int(blue, 16))
print('Initialized', len(ebugs), 'eBug(s)')
