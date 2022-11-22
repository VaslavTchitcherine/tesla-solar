#!/usr/bin/python
#
# solar.py
# Test reading ACS758 hall effect linear current sensors on IN0 and IN2 of ADS1115
#

import time
import Adafruit_ADS1x15

# create an ADS1115 ADC (16-bit) instance,
# default I2C address and bus
adc = Adafruit_ADS1x15.ADS1115(address=0x48, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V	1 bit is 0.125mV
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V

start = time.time()

while True:
	# volts
        batt_curr = adc.read_adc(0, gain=1)
        panel_curr = adc.read_adc(2, gain=1)
    	print time.time()-start, batt_curr, panel_curr
    	#time.sleep(0.1)
