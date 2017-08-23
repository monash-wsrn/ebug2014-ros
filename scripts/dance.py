#!/usr/bin/python2
import time

from nrf import bridge
nrf = bridge()
swarm = nrf.assign_addresses()
for i,j in swarm.items():
    print(i,j)

print("I am going to use address 2:")

nrf.set_TX_address(2)
nrf.enable_LEDs(0,1,1,1)
nrf.LED_brightness(4)

time.sleep(10)
nrf.set_LEDs(0x4e30,0x9341,0x208e)

for i in range(1,4):
    time.sleep(0.5)
    nrf.enable_LEDs(0,0,1,1)
    time.sleep(0.5)
    nrf.enable_LEDs(0,1,1,1)

for i in range(1,4):
    nrf.motor_control(400,400,0)
    time.sleep(3)
    nrf.motor_control(-400,-400,0)
    time.sleep(3)

nrf.enable_LEDs(0,0,1,1)
nrf.motor_control(0,0,0)
