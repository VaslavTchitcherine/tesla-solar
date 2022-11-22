#!/usr/bin/python
#
# unyank.py
# Turn on both SSRs
#

from gpiozero import LED	# for GPIO control of SSR disconnects

# GPIO lines to control the load and solar panel SSR connects
panel_connect = LED(5)	# GPIO 5 is pin 29 on the Pi header
load_connect = LED(6)	# GPIO 6 is pin 31 on the Pi header

# load and solar panels are both disconnected until we are sure voltages are OK
load_connect.on()
panel_connect.on()
