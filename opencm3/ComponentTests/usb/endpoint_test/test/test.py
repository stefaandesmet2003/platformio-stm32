#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep  7 20:08:17 2021

@author: stefaan
"""

# test usb protocol for ST-LINK swim mode
# pip install pyusb indien usb. niet in de huidige env aanwezig
# dit is gewoon een dump van de commands op de repl
# mogelijk werkt dit niet bij run omwille van 'busy' replies van STM8
# je moet cmd_read_status doen totdat je een OK=0 terugkrijgt

import usb.core
import usb.util
import time #for sleep

dev = usb.core.find(idVendor=0xcafe)
# wat functies uit pyusb, maar niet nodig voor het vervolg
cfg = dev.get_active_configuration()
# de enige interface zit nu op cfg[(0,0)]
intf = cfg[(0,0)]
# ep 1 : 0x81, bulk in
# ep 2 : 0x02, bulk out
ep0 = intf[0]

for i in range(5):
    dev.write(1,"tic")
    tic_val = dev.read(0x82,1)
    dev.write(1,"toc")
    toc_val = dev.read(0x82,1)
    time.sleep(1.0)
    print(f"tic={tic_val[0]}, toc={toc_val[0]}")
dev.reset() # dit lijkt dev te releasen ook, anders 'resource busy' error


