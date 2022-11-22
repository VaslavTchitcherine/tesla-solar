#!/usr/bin/python
#
# gateblink.py
# Test reading hall sensors on IN0 and IN2 of ADS1115, while blinking mosfets
#

from time import sleep
import Adafruit_ADS1x15
from gpiozero import LED

# create an ADS1115 ADC (16-bit) instance
adc = Adafruit_ADS1x15.ADS1115()

# to MCP1407 MOSFET drivers
panelswitch = LED(5)
batteryswitch = LED(6)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V

while True:
	print 'on'
        panelswitch.on()
        batteryswitch.on()
	panels = adc.read_adc(0, gain=1)
	battery = adc.read_adc(2, gain=1)
	print panels,battery
        sleep(1)

        print 'off'
        panelswitch.off()
        batteryswitch.off()
	panels = adc.read_adc(0, gain=1)
	battery = adc.read_adc(2, gain=1)
	print panels,battery
        sleep(1)
