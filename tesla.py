#!/usr/bin/python
#
# tesla.py
#
# Pi Zero W monitor for homebrew solar system using Tesla modules.
# 1) Read and log voltage and temperature data from the Tesla modules.
# 2) A/D conversion of battery and solar panel current hall sensors.
# 3) GPIO for solid state relays for battery and solar panel connects.
# Mandatory command line positional arguments are:
# 	log file (for web display by dygraph.js)
# 	log file with all 12 cell voltages
#	log file for errors
# Example:
#	nohup tesla.py /var/www/html/log.csv /tmp/cellvolts /tmp/error &
#

import serial
import time
import math
import sys
import select
import argparse
import Adafruit_ADS1x15		# 16 bit A/D for hall current sensors
from gpiozero import LED	# for GPIO control of SSR disconnects
import atexit

# register values for tesla bms boards
REG_ADDR = 0		# board address [1,62] is at register 0
REG_GPAI = 1		# general purpose i/o
REG_ALERT_STATUS = 0x20
REG_ADC_CTRL  = 0x30
REG_IO_CTRL   = 0x31
REG_BAL_CTRL  = 0x32	# to set balance timeout (?)
REG_BAL_TIME  = 0x33	# to set state of balancing resistors
REG_ADC_CONV  = 0x34
REG_ADDR_CTRL = 0x3B

# voltage threshold at which balancing begins (WHAT SHOULD THIS BE?)
balancethresh = 0.1

# log record emitted how frequently (sec)
loginterval = 5

# epoch time when last log record emitted
lastemit = 0

# exponential moving average smoothing period
emaperiod = 10		# 1 for no smoothing
emafactor = 2.0/(emaperiod+1.0)

# initialization of the ema-smoother values
batt_current = 0
panel_current = 0

# module voltage extrema (for state of charge computation)
#maxv = 24.6;   # 24.6V is 100% SoC (4.1V per cell)
maxv = 24.0;
###minv = 18.6;   # 18.6V is 0% SoC (3.1V per cell)
minv = 19.8;   # 19.8V is 0% SoC (3.3V per cell)

# called when terminated with ^C or kill
@atexit.register
def goodbye():
	print "Terminated"
	# turn off both SSRs
	load_connect.off()
	panel_connect.off()

# log to error file
def warn(s):
	print("WARNING: {}".format(s))
	logerrorsfile.write(time.strftime("%Y-%m-%d %H:%M:%S "))
	logerrorsfile.write(s)

def hex(arr):
	return(''.join(' {:02x}'.format(x) for x in arr))

# convert raw thermistor data to degrees C
def tempCalc(b1,b2):
	t = 1000 * (1.78*float(33046) / (int(b1) * 256 + int(b2) + 2)  - 3.57)
	if t <= 0:
		warn("Bad temperature reading: b1={}, b2={}, val = {}".format(b1,b2,t))
	t = 1 / (0.0007610373573 + 0.0002728524832 * math.log(t) + 0.0000001022822735 * (math.log(t)**3))
	return(t - 273.15)		  

# return crc byte for a specified byte array
# Note: genCRC([x1, x2, ... xn]) == 0 iff genCRC([x1,x2,...xn-1]) == xn
def genCRC(data):
	generator = 7
	crc = 0
	for b in data:
		crc = crc ^ b 
		for i in range(8):
			if (crc & 0x80) != 0:
				crc = ((crc << 1) ^ generator) & 255
			else:
				crc <<= 1
	return(crc) 

# class for one module
class BMSBoard:
	def __init__(self, bus, address):
		self.bus = bus
		self.address = address
		self.balancemask = 0		# bitmask for the 6 balancing resistors
		#log("Created BMSBoard {}.".format(address))

	def __str__(self):
		s = "BMS Board {} ".format(self.address)
		s += "Alerts: {}, Faults: {} ".format(self.alerts, self.faults)
		s += ", Cell Over Voltage Faults: {}, Under: {} ".format(self.cov_faults, self.cuv_faults)
		s += ", Cell Voltages = "+"".join("{0:.4f}V ".format(v) for v in self.cellVolt)
		s += ", Temps = {0:.2f}C {1:.2f}C".format(self.temperatures[0],self.temperatures[1])
		return s

	# Called to get new values (voltages, temperatures) from a module.
	# Also reads status, we probably don't care about this, as our program monitors module parameters.
	def update(self):
		self.readStatus()
		self.readModuleValues()
		# total voltage of this module
		self.moduleVolt = self.cellVolt[0] + self.cellVolt[1] + self.cellVolt[2] + self.cellVolt[3] + self.cellVolt[4] + self.cellVolt[5];
		# compute state of charge using voltage extrema
		self.soc = 100 * (self.moduleVolt - minv) / (maxv - minv)
		self.balance();		# activate balancing resistors as necessary
		#log("Board {} status: {}".format(self.address,self))
		# check voltages and set SSRs appropriately
		if ( self.moduleVolt >= minv and self.moduleVolt <= maxv ):
			load_connect.on()
			panel_connect.on()
		# low voltage, disconnect load but keep panels connected
		elif ( self.moduleVolt < minv ):
			warn('low voltage disconnect, load off')
			load_connect.off()
			panel_connect.on()
		# high voltage, disconnect panels but keep load connected
		elif ( self.moduleVolt > maxv ):
			warn('high voltage disconnect, panels off')
			load_connect.on()
			panel_connect.off()

	# dump board data to log files
	def dump(self):
		logfile.write("{0:.2f},{1:.4f},{2:.1f},{3:.1f},{4:.6f},".format(self.soc,self.moduleVolt,self.temperatures[0],self.temperatures[1],self.imbalance))
		# also write cell voltages to another log file
		logcellsfile.write("{0:.4f} {1:.4f},{2:.4f} {3:.4f} {4:.4f} {5:.4f} ".format(self.cellVolt[0],self.cellVolt[1],self.cellVolt[2],self.cellVolt[3],self.cellVolt[4],self.cellVolt[5]))

	# Determine the mask for balancing resistors
	# The resistor is switched in for a cell if its voltage is sufficiently higher than all others.
	# Only one resistor will ever be switched in at any given time.
	def balance(self):
		# determine highest cell voltage and its location
		high = 0.0
		for i in range(6):
			if ( self.cellVolt[i] > high ):
				high = self.cellVolt[i]
				highat = i;
		# determine 2d highest cell voltage
		high2 = 0.0;
		for i in range(6):
			if ( self.cellVolt[i] > high2 and i != highat ):
				high2 = self.cellVolt[i]

		# imbalance is difference between highest and second highest cell
		self.imbalance = high-high2

		# is the imbalance sufficiently large to bleed it?
		mask = 0;
		if ( self.imbalance > balancethresh ):
			mask = 1<<(5-highat);
		# if the mask has changed, set state of resistors
		if ( mask != self.balancemask ):
			self.setbalance(mask)
			self.balancemask = mask		

	# Called with a 6 bit mask to set balancing resistors on/off
	def setbalance(self,mask):
		# 5 second balance timeout
		# if not triggered to balance it will stop after 5 seconds  (???)
		# or will or only balance for 5 seconds?
		self.send_and_receive_reply(REG_BAL_TIME, 5, True)
		# send the mask
		self.send_and_receive_reply(REG_BAL_CTRL, mask, True) 
		# XXX readback register values to be sure they got set???

	def send_and_receive_reply(self, command, param, isWrite = False):
		return(self.bus.send_and_receive_reply_to(self.address,command, param, isWrite))

	# poll for status of a module
	def readStatus(self):
		r = self.send_and_receive_reply(REG_ALERT_STATUS,4)
		self.alerts,self.faults,self.cov_faults, self.cuv_faults = r[3:7]

	# poll for values (voltages, temperatures) from a module
	def readModuleValues(self):
		# ADC Auto mode, read every ADC input we can (Both Temps, Pack, 6 cells)
		# sets which channels we read
		self.send_and_receive_reply(REG_ADC_CTRL, 0b00111101, True)		# 0x3D
		# enable temperature measurement VSS pins
		self.send_and_receive_reply(REG_IO_CTRL, 0b00000011, True)		# 0x03
		# start all ADC conversions		
		self.send_and_receive_reply(REG_ADC_CONV, 1, True)
		# request the 18 byte (0x12) data record
		r = self.send_and_receive_reply(REG_GPAI,0x12, False)

		# we expect to have read 22 bytes, the 18 byte data record plus:
		#	3 bytes of header (address, command, length)
		#	1 byte footer (crc)
		if (len(r) != 22):
			warn("readModuleValues expected 22 bytes but got {}".format(len(r)))
			return

		# payload is 2 bytes gpai, 2 bytes for each of 6 cell voltages, 
		# 2 bytes for each of two temperatures (18 bytes of data)
		self.cellVolt = [(r[5 + (i * 2)] * 256 + r[6 + (i * 2)]) * 0.000381493 for i in range(6)]
		self.temperatures = [tempCalc(r[17],r[18]),tempCalc(r[19],r[20])]


# class for the entire set of up to 63 modules
class BMSBus:
	def __init__(self):
		self.findBoards()
	
	# see if any module [1,62] responds
	def findBoards(self):
		self.boards = []
		for i in range(1,62):
			# read address of module i, one byte
			r = self.send_and_receive_reply_to(i,REG_ADDR,1,False)
			if len(r) != 0:
				if not (r[0] == 2*i and r[1] == 0 and r[2] == 1):
					print("Unexpected response!")
				if len(r) > 3:
					if not (r[4] > 0) :
						warn("r4 was = {}".format(r[4]))
					else:
						print("BMSBoard {} responded {}".format(i,hex(r[3:])))
						self.boards.append(BMSBoard(self,i))
			time.sleep(0.005)

	# PERHAPS WARN IF UNEXPECTED # OF BOARDS RESPONDED?

	# update status and values for all boards
	def updateBoards(self):
		for board in self.boards:
			board.update()

	# dump values for all boards to log files
	def dumpBoards(self):
		for board in self.boards:
			board.dump()

	# initialize all boards:
	#	balancing resistors off
	#	clear alerts()?
	# NEEDED?
	def resetBoards(self):
		for board in self.boards:
			board.setbalance(0);

	def send_and_receive_reply_to(self, destination, command, param, isWrite = False):
		# huh?  The dest changes if isWrite ???
		dest = 2*destination +1 if isWrite else 2*destination
		payload = bytearray([dest, command, param])
		if isWrite: 
			# original code was doing weird things that I think amounted to nothing. So I simplified.
			crc = genCRC(payload)
			###log("isWrite, therefore adding CRC = {}".format(crc))
			payload.append(crc)
		ser.write(payload)
		#log("Sent: {}".format(hex(payload)))
		time.sleep(0.02)
		reply=[]
		while(ser.inWaiting()):
			reply.append(ord(ser.read()))
		# check crc
		if isWrite and genCRC(reply) != 0:
			warn("CRC error: Reply was {}".format(hex(reply)))
		return(reply)

##########################
# main program

# check for correct number of mandatory command line arguments
if ( len(sys.argv) != 4 ):
        print "Usage: tesla.py <main_logfile> <cellvolts_logfile> <error_logfile>"
        sys.exit(-1)

# open the serial port, hardware serial GPIO14 and GPIO15 on Pi Zero W.
# Must use raspi-config, Interfacing Options, P6 to disable console and allow access.
# Also add the line:
#	dtoverlay=pi3-disable-bt
# to /boot/config.txt so the PL011 UART is mapped to GPIO14 and GPIO15 and appears as /dev/serial0
try:
	ser = serial.Serial( # set parameters
		port='/dev/serial0',	# for hardware serial port (GPIO14 and GPIO15 on Pi Zero W)
		#port='/dev/ttyUSB0',	# for adafruit ftdi cable
		baudrate=612500,
		timeout=1
	)
	ser.isOpen() # try to open port

except IOError: # if port is already opened, close it and open it again
	try:
		ser.close()
		ser.open()
		print ("port was already open, was closed and opened again")
	except: #ser object doesn't even exist? must be a port setting fault
		ser=False
		print ("Check port settings!")
		exit(-1);

# parse command line arguments 
parser = argparse.ArgumentParser(description='tesla solar controller')
parser.add_argument("log")
parser.add_argument("logcells")
parser.add_argument("logerrors")
args = parser.parse_args()

# try to open all 3 log files for write (append?)
logfile = open(args.log, 'w')
logcellsfile = open(args.logcells, 'w')
logerrorsfile = open(args.logerrors, 'w')

# find all the modules, also calls update once for each
bms = BMSBus()

# balancing resistors off, clear alerts
bms.resetBoards()

# create an ADS1115 ADC (16-bit) instance
adc = Adafruit_ADS1x15.ADS1115()

# GPIO lines to control the load and solar panel SSR connects
panel_connect = LED(5)	# GPIO 5 is pin 29 on the Pi header
load_connect = LED(6)	# GPIO 6 is pin 31 on the Pi header

# colors of the indicator LED
led_red = LED(19)
led_green = LED(26)
led_blue = LED(13)

# indicator LED state
led_green_on = 0

# load and solar panels are both disconnected until we are sure voltages are OK
load_connect.off()
panel_connect.off()

while True:
	# read boards, update internal state
	bms.updateBoards()

	# local time (in a format that js can parse)
	timestring = time.strftime("%Y-%m-%d %H:%M:%S")

	# write to log file
	logfile.write(timestring)
	logfile.write(',')

	# write to cell-level log file
	logcellsfile.write(timestring)
	logcellsfile.write(' ')

	# dump data from boards (to both log files)
	bms.dumpBoards()

	# Read ACS758 hall sensor(s) to obtain currents
	# Subtract a/d offset, scale into amps, and ema filter
	# (THE 0.01786 COEFFICIENT TO SCALE INTO AMPS IS APPROXIMATE, SHOULD RECALIBRATE THIS)
        rawadc = 0.01786 * (adc.read_adc(0, gain=1) - 8681)
        batt_current = emafactor * rawadc + (1.0 - emafactor) * batt_current

	# write battery current to log file (negated)
	logfile.write("{0:.3f}".format(-batt_current))

	# newlines and flushing of logfiles
	logfile.write("\n")
	logcellsfile.write("\n")
	logfile.flush()
	logcellsfile.flush()
	logerrorsfile.flush()

	# current epoch time
	now = time.time()

	# inter-record delay
	wait = loginterval - (now - lastemit)

	if ( wait > 0 ):
		time.sleep(wait)

	lastemit = time.time()

	# toggle status LED
	if ( led_green_on ):
		led_green_on = 0
		led_green.off()
	else:
		led_green_on = 1
		led_green.on()

