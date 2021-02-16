#Reading data from the Sharp GP2Y0E02B IR sensor with Rasp Pi

import smbus
import math
import pigpio

pi = pigpio.pi()

#####Registers#####

#Device 
bus = smbus.SMBus(1)
IR_REG = 0x40          #Taken from i2cdetect

Dist1 = 0x5E          #Distance[11:4]
Dist2 = 0x5F          #Distance[3:0]
Shift_Bit = 0x35      #Shift_Bit[2:0]


####Data Reading####
def openI2C():
	global handle
	handle = pi.i2c_open(1, IR_REG, 0) #Open device at address 0x40 on bus 1
	return handle

def dataRead(adr):
	return bus.read_byte_data(IR_REG, adr)

def distCalc():
	global Distance
	D1 = dataRead(Dist1)
	D2 = dataRead(Dist2)
	SBit = dataRead(Shift_Bit)
	DisValue = ((((D1 * 16) + D2) / 16) / math.pow(2, SBit))
	Distance = DisValue
	return Distance

def closeI2C():
	pi.i2c_close(handle) #close device
