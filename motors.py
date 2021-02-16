#Controlling the motors with the TB6612FNG Motor Driver

import pigpio
import time
import math
import odometry
import robot_main

pi = pigpio.pi()

#Define motor driver pins
STBY = 23
PWMA = 20
PWMB = 27
AIN1 = 22
AIN2 = 21
BIN1 = 24
BIN2 = 25

# Set motor driver pins as outputs
pi.set_mode(STBY, pigpio.OUTPUT)
pi.set_mode(PWMA, pigpio.OUTPUT)
pi.set_mode(PWMB, pigpio.OUTPUT)
pi.set_mode(AIN1, pigpio.OUTPUT)
pi.set_mode(AIN2, pigpio.OUTPUT)
pi.set_mode(BIN1, pigpio.OUTPUT)
pi.set_mode(BIN2, pigpio.OUTPUT)

#Encoder Outputs
AENC1 = 18
AENC2 = 19
BENC1 = 17
BENC2 = 16

#Global variables for encoder counts
countR = 0
countL = 0

#Variable to be assigned for motors after PWM duty cycle calculated in duty_cycle()
pwm_cycle = 0

#speed in cm/s
speed = 10

#Function to determine the PWM duty cycle depending on the desired speed in cm/s
def duty_cycle(speed):
	global pwm_duty_cycle
	pwm_duty_cycle = (speed + 0.361)/0.058
	return pwm_duty_cycle


def pulseA(gpio, level, tick):
	global countR
	countR += 1


def pulseB(gpio, level,  tick):
	global countL
	countL += 1	

#Interupt handlers for the encoders
callback1 = pi.callback(AENC1, pigpio.RISING_EDGE, pulseA)
callback2 = pi.callback(BENC1, pigpio.RISING_EDGE, pulseB)


###### Movement Functions ######

""" Function to Initialise robot motors and boolean variables"""
def initialise():
	pi.write(STBY, 1)
	pi.set_PWM_dutycycle(PWMA, 0)
	pi.set_PWM_dutycycle(PWMB, 0)

	#Setting all movement flags to False
	Forward.has_been_called = False
	backward.has_been_called = False
	left.has_been_called = False
	right.has_been_called = False
	forward_arc.has_been_called = False

""" Function to disbale robot motors"""
def disengage():
	pi.write(STBY, 0)
	pi.set_PWM_dutycycle(PWMA, 0)
	pi.set_PWM_dutycycle(PWMB, 0)

"""Class to control forward motion"""
class Forward(object):
	def __init__(self, velocity):
		self.pwm_cycleA = duty_cycle(velocity) + 2
		self.pwm_cycleB = duty_cycle(velocity) 

	#Function to drive straight ahead
	def forward(self):
		#ccw
		Forward.has_been_called = True
		
		pi.set_PWM_dutycycle(PWMA, self.pwm_cycleA)
		pi.set_PWM_dutycycle(PWMB, self.pwm_cycleB)
		pi.write(AIN1, 0)
		pi.write(AIN2, 1)
		pi.write(BIN1, 1)
		pi.write(BIN2, 0)

#Move robot forward a certain distance
class Dist_forward(Forward):
	def __init__(self):
		super().__init__(speed)

	def move_dist(self, distance):
		super().forward()

		if (Forward.has_been_called):	
			print("hi")
			if odometry.distanceA and odometry.distanceB >= distance:
				Forward.has_been_called = False
				print("ho")
				brake()
				time.sleep(1)
				odometry.reset_variables()

#Move robot forward for a given amount of time
class Time_forward(Forward):
	def __init__(self):
		super().__init__(speed)

	def move_time(self, seconds):
		super().forward()
		
		distance = seconds*speed

		if (Forward.has_been_called):	
			if odometry.distanceA and odometry.distanceB >= distance:
				Forward.has_been_called = False
				brake()
				time.sleep(1)
				odometry.reset_variables()

"""Function to arc around an object or gently turn"""
def forward_arc(velocityA, velocityB, angle):

	forward_arc.has_been_called = True

	pwm_cycleA = duty_cycle(velocityA)
	pwm_cycleB = duty_cycle(velocityB)

	pi.set_PWM_dutycycle(PWMA, pwm_cycleA)
	pi.set_PWM_dutycycle(PWMB, pwm_cycleB)
	pi.write(AIN1, 0)
	pi.write(AIN2, 1)
	pi.write(BIN1, 1)
	pi.write(BIN2, 0)
	
""" Function to move backwards """
def backward(speed):
	#cw
	backward.has_been_called = True

	pwm_cycle = duty_cycle(speed)

	pi.set_PWM_dutycycle(PWMA, pwm_cycle)
	pi.set_PWM_dutycycle(PWMB, pwm_cycle)
	pi.write(AIN1, 1)
	pi.write(AIN2, 0)
	pi.write(BIN1, 0)
	pi.write(BIN2, 1)


""" Function to turn left """
def left(angle):
	#right wheel ccw, left wheel cw
	left.has_been_called = True
	pi.write(AIN1, 1)
	pi.write(AIN2, 0)
	pi.write(BIN1, 1)
	pi.write(BIN2, 0)

	#get target encoder count and distance for the turn movement
	odometry.pivot_turn(angle)

""" Function to turn right """
def right(angle):
	#right wheel cw, left wheel ccw
	right.has_been_called = True
	pi.write(AIN1, 0)
	pi.write(AIN2, 1)
	pi.write(BIN1, 0)
	pi.write(BIN2, 1)

	#get target encoder count and distance for the turn movement
	odometry.pivot_turn(angle)
	

""" Function to short brake """
def brake():
	#short break
	pi.write(AIN1, 1)
	pi.write(AIN2, 1)
	pi.write(BIN1, 1)
	pi.write(BIN2, 1)

""" Function to stop completely """
def stop():
	#Stop
	pi.write(AIN1, 0)
	pi.write(AIN2, 0)
	pi.write(BIN1, 0)
	pi.write(BIN2, 0)


