#!/usr/bin/python
#
# fastamps.py
# Read and dump battery current as fast as the a/d permits.
# Probably a bad idea to run this at the same time as tesla.py.
# Note:  Conversion to amps is approximate, the
# hall sensor was calibrated from the power display on the
# inverter, which is likely not very accurate.
#

import time

# Import the ADS1x15 module.
import Adafruit_ADS1x15

# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1

# Start continuous ADC conversions on channel 0 using the previously set gain
# value.  Note you can also pass an optional data_rate parameter, see the simpletest.py
# example and read_adc function for more infromation.
adc.start_adc(0, gain=GAIN)
# Once continuous ADC conversions are started you can call get_last_result() to
# retrieve the latest result, or stop_adc() to stop conversions.

# Note you can also call start_adc_difference() to take continuous differential
# readings.  See the read_adc_difference() function in differential.py for more
# information and parameter description.

# Read and dump channel 0 continuously
start = time.time()
while ( 1 ):
    # read the last ADC conversion value
    value = adc.get_last_result()
    # convert to amps
    value -= 13203
    value *= 0.0174
    print time.time()-start, value

# Stop continuous conversion.  After this point you can't get data from get_last_result!
adc.stop_adc()
