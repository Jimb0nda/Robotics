# Holds speed, distance calculations and variables

import time
import math
import motors
import robot_main

#Wheel circumference
whCirc = 5*math.pi

#Diameter of robots pivot
d = 21

#Number of ticks per wheel revolution
tickPerRev = 894

#Distance of wheel rotation per encoder tick
wheelDist = whCirc/tickPerRev

#Circumference of robot pivot
pivotCirc = math.pi * d

#Global variables for speed
speedA = 0
speedB = 0

#Global variables for distance
distanceA = 0
distanceB = 0

"""Arc around an object or gently turn with a given radius"""
def arc_turn(velocity, angle, radius):
	
	#global targetCount
	v = velocity
	x = angle
	r = radius + 10.5
	d = 10.5

	global arcDistL
	global arcDistR
	global arcTargetCountL
	global arcTargetCountR

	#Calculating the velocity of the slower wheel
	v_slow = (v*(r - d)) / (r + d)  

	#If right flag has been set, set the left wheel to be slower and vice versa
	if(motors.right.has_been_called):
		motors.right.has_been_called = False

		arcDistL = math.pi*(r-10.5)
		arcDistR = math.pi*(r+10.5)

		#based on distance, calculate number of wheel revolutions
		numRevL = arcDistL / whCirc
		numRevR = arcDistR / whCirc

		#based on number of revolutions, calculate target encoder count
		arcTargetCountL = numRevL * 792
		arcTargetCountR = numRevR * 792

		motors.forward_arc(v_slow, v, x)
		
	elif(motors.left.has_been_called):
		motors.left.has_been_called = False

		arcDistL = math.pi*(r+10.5)
		arcDistR = math.pi*(r-10.5)

		#based on distance, calculate number of wheel revolutions
		numRevL = arcDistL / whCirc
		numRevR = arcDistR / whCirc

		#based on number of revolutions, calculate target encoder count
		arcTargetCountL = numRevL * 894
		arcTargetCountR = numRevR * 894

		motors.forward_arc(v, v_slow, x)
		
"""Turn on the spot"""
def pivot_turn(angle):

	global distance
	global targetCount

	#based on angle, calculate distance (arc length) for pivot
	distance = abs(angle) / 360.0 * pivotCirc

	#based on distance, calculate number of wheel revolutions
	numRev = distance / whCirc

	#based on number of revolutions, calculate target encoder count
	targetCount = numRev * tickPerRev

	
"""Function to reset encoder count and distance travelled variables"""
def reset_variables():
	global distanceA
	global distanceB

	motors.countR = 0
	motors.countL = 0
	distanceA = 0
	distanceB = 0




