#!/usr/bin/python3

from threading import Thread
import enum

import time
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html
import math
import sys
import select
import tty
import termios
import random
import numpy as np

# Import the necessary camera packages
import picamera
import picamera.array
import cv2

#Import other modules
import IRsensor
import motors
import odometry
import vision
import states
import astar_algo

pi = pigpio.pi()

# Global Flags
RUNNING = 1			# Main loop running
irSent = 1
vis_sleep = 0
going_home = 0
finished = 0

stop_threads = 0 	# stop threads when set high

# Global variables
ir = 0            											# Infrared sensor value from sensor
maze = [[-1 for col in range(20)] for row in range(10)]		# Initial matrix setup for the map
start_position = [5,1]										# Starting position of robot relative to the map
current_position = [5,1]									# Current Position of the robot
obstacle_infront = [0,0]									# If Obstacle infront, Update and add to map
path = []													# List of coordiantes for route back


# Function for IR sensor to be ran in new thread
def sensor():
	global ir
	global irSent

	while(RUNNING):
		if (irSent == 0):					# If IRsent is set to 0, sensor must sleep for ability to turn accruately from point of contact
			time.sleep(2)
			irSent = 1						# Start taking readings again
		elif(stop_threads):
			break
		else:
			IRsensor.distCalc() 			# Use distance calculation function for the IR sensor
			ir = IRsensor.Distance 			# Assign distance value

# Function for camera to be ran in new thread
def camera():
	global start_flag
	global finish_flag
	global vis_sleep

	camera = picamera.PiCamera()

	while(RUNNING):
		if (vis_sleep == 1):					# If vis_sleep is set to 1, camera to close as no longer needed after finding end goal
			camera.close()
		elif(stop_threads):
			break
		else:
			vision.aruco_detection(camera)
			finish_flag = vision.finish_flag	# Set Finish flag if in view of robot
			time.sleep(1)


def main():
	try:
		global RUNNING
		global stop_threads
		global irSent
		global going_home
		global path
		global maze
		global start_position
		global current_position
		global obstacle_infront
		global east_flag
		global west_flag
		global south_flag
		global north_flag
		global vis_sleep
		global finished

		east_flag = 0		# Directional flags for navigation
		west_flag = 0
		south_flag = 0
		north_flag = 1
		
		left = 0			# Directional flags for direction returning home
		right = 0
		forward = 1

		# Initialise time and encoder interrupts before main loop
		starttime = time.time()
		motors.initialise()
		motors.callback1
		motors.callback2
		IRsensor.openI2C() #Open device
		
		# Starting Threads
		sensorThread.start()
		visionThread.start()
		
		# Set the robot off searching
		states.searching()

		while(RUNNING):

			#Distance
			odometry.distanceA = odometry.wheelDist * motors.countR
			odometry.distanceB = odometry.wheelDist * motors.countL

			# Tracking elapsed time
			elapsedtime = time.time() - starttime

			# If IR sensor detects an object, place a marker within the map ahead of the current position
			if (irSent == 1 and ir < 10):
				irSent = 0											#Set flag to 0 to pause the IR sensor to allow it to turn eactly 90 from point of contact
				motors.Forward.has_been_called = False
				motors.right.has_been_called = False
				motors.left.has_been_called = False
				if (north_flag == 1):								# If going North, place obstacle marker ahead one space on map
					obstacle_infront[0] = current_position[0]
					obstacle_infront[1] = current_position[1] + 1
					maze[obstacle_infront[0]][obstacle_infront[1]] = 1
				elif(south_flag == 1):								# If going South, place obstacle marker behind one space on map
					obstacle_infront[0] = current_position[0] 
					obstacle_infront[1] = current_position[1] - 1
					maze[obstacle_infront[0]][obstacle_infront[1]] = 1
				elif(east_flag == 1):								# If going East, place obstacle marker left one space on map
					obstacle_infront[0] = current_position[0] + 1
					obstacle_infront[1] = current_position[1]
					maze[obstacle_infront[0]][obstacle_infront[1]] = 1
				elif(west_flag == 1):								# If going West, place obstacle marker right one space on map
					obstacle_infront[0] = current_position[0] - 1
					obstacle_infront[1] = current_position[1]
					maze[obstacle_infront[0]][obstacle_infront[1]] = 1
				odometry.reset_variables()							# reset encoders and distance counts
				states.avoidance()									# avoid obstacle

			#--------------Going East----------------
			elif (motors.Forward.has_been_called and east_flag == 1): 
				if odometry.distanceA and odometry.distanceB >= 22:
					print("East Bound")
					maze[current_position[0]][current_position[1]] = 0
					current_position[0] = current_position[0]+1
					maze[current_position[0]][current_position[1]] = 0
					odometry.reset_variables()
					motors.right.has_been_called = False
					east_flag = 1
			#--------------Going West----------------
			elif (motors.Forward.has_been_called and west_flag == 1): 
				if odometry.distanceA and odometry.distanceB >= 22:
					print("West Bound")
					maze[current_position[0]][current_position[1]] = 0
					current_position[0] = current_position[0]-1
					maze[current_position[0]][current_position[1]] = 0
					odometry.reset_variables()
					motors.left.has_been_called = False
			#--------------Going North----------------
			elif (motors.Forward.has_been_called and north_flag == 1): 
				if odometry.distanceA and odometry.distanceB >= 22:
					print("North Bound")
					maze[current_position[0]][current_position[1]] = 0
					current_position[1] = current_position[1]+1
					maze[current_position[0]][current_position[1]] = 0
					odometry.reset_variables()
			#--------------Going South----------------
			elif (motors.Forward.has_been_called and south_flag == 1):
				if odometry.distanceA and odometry.distanceB >= 22:
					print("South Bound")
					motors.right.has_been_called = False
					motors.left.has_been_called = False
					maze[current_position[0]][current_position[1]] = 0
					current_position[1] = current_position[1]-1
					maze[current_position[0]][current_position[1]] = 0
					odometry.reset_variables()
			#--------------Turning Right----------------
			elif (motors.right.has_been_called and not finished):
				if(motors.countR >= odometry.targetCount):
					print("Turned Right")
					odometry.reset_variables()
					if(north_flag == 1):
						motors.right.has_been_called = False
						north_flag = 0
						east_flag = 1
					elif(east_flag == 1):
						motors.right.has_been_called = False
						south_flag = 1
						east_flag = 0
					elif(west_flag == 1):
						motors.right.has_been_called = False
						north_flag = 1
						west_flag = 0
					elif(south_flag == 1):
						motors.right.has_been_called = False
						south_flag = 0
						west_flag = 1
					states.searching()
			#--------------Turning Left----------------
			elif (motors.left.has_been_called):
				if(motors.countL >= odometry.targetCount):
					print("Turned Left")
					odometry.reset_variables()
					if(north_flag == 1):
						motors.left.has_been_called = False
						north_flag = 0
						west_flag = 1
					elif(east_flag == 1):
						motors.left.has_been_called = False
						north_flag = 1
						east_flag = 0
					elif(west_flag == 1):
						motors.left.has_been_called = False
						south_flag = 1
						west_flag = 0
					elif(south_flag == 1):
						motors.left.has_been_called = False
						south_flag = 0
						east_flag = 1
					states.searching()

			# Reached the finish aruco tag
			if (vision.finish_flag == 1 and vision.aruco_dist <= 30):
				vision.aruco_dist = 100     				# Sets distnace to far away due to confusion while turning
				vis_sleep = 1								# Set the camera to sleep to allow it to turn from point of contact
				irSent = 0									# Set IR sensor to sleep
				vision.finish_flag == 0
				motors.Forward.has_been_called = False

				# Pass map into A Star algorithm to return shortest path back to the starting place and print path
				path = astar_algo.astar(maze, tuple(start_position), tuple(current_position))
				print("original path = {} ".format(path))
				print(np.array(maze))	# Print created map	
				print("Robot Position finish position = {} ".format(current_position))
				path.pop()   			# Pop the current robot position off the list of coordinates ready for returning
				finished = 1
				odometry.reset_variables()
				motors.right(180)

			#Turning around once at the finish aruco tag
			if (motors.right.has_been_called and finished == 1):
				if(motors.countR >= odometry.targetCount):
					motors.right.has_been_called = False
					finished = 0
					going_home = 1
					odometry.reset_variables()
					motors.disengage()

			if (going_home):
				motors.initialise()
				if len(path) > 0:
					# path coordinates in (x,y)
					# If x is same but y is less, move forward
					if (path[-1][1] < current_position[1] and path[-1][0] == current_position[0] and forward):
						current_position = path[-1]
						print("Path left to travel = {} ".format(path))
						print("Move Forward")
						print("Robot Position = {} ".format(current_position))
						path.pop()
					# If x is same but y is greater, move forward/backwards away from starting point
					elif (path[-1][1] > current_position[1] and path[-1][0] == current_position[0] and forward):
						current_position = path[-1]
						print("Path left to travel = {} ".format(path))
						print("Move Forward")
						print("Robot Position = {} ".format(current_position))
						path.pop()
					# If x is less but y is same, turn and then move forward
					elif(path[-1][1] == current_position[1] and path[-1][0] < current_position[0] and forward):
						forward = 0
						right = 1
						current_position = path[-1]
						print("Path left to travel = {} ".format(path))
						print("Turn and Move Forward")
						print("Robot Position = {} ".format(current_position))
						path.pop()
					# If x is greater but y is same, turn and then move forward
					elif(path[-1][1] == current_position[1] and path[-1][0] > current_position[0] and forward):
						forward = 0
						left = 1
						current_position = path[-1]
						print("Path left to travel = {} ".format(path))
						print("Turn and Move Forward")
						print("Robot Position = {} ".format(current_position))
						path.pop()
					# If x is same but y is less and you have already turned before, turn and then move forward
					elif(path[-1][1] < current_position[1] and path[-1][0] == current_position[0] and right):
						forward = 1
						right = 0
						current_position = path[-1]
						print("Path left to travel = {} ".format(path))
						print("Turn and Move Forward")
						print("Robot Position = {} ".format(current_position))
						path.pop()
					# If x is less but y is same and you have already turned before, move forward
					elif(path[-1][1] == current_position[1] and path[-1][0] < current_position[0] and right):
						forward = 0
						right = 1
						current_position = path[-1]
						print("Path left to travel = {} ".format(path))
						print("Move Forward")
						print("Robot Position = {} ".format(current_position))
						path.pop()
					# If x is greater but y is same and you have already turned before, move forward
					elif(path[-1][1] == current_position[1] and path[-1][0] > current_position[0] and left):
						forward = 0
						left = 1
						current_position = path[-1]
						print("Path left to travel = {} ".format(path))
						print("Move Forward")
						print("Robot Position = {} ".format(current_position))
						path.pop()
					# If x is same but y is less and you have already turned before, turn and then move forward
					elif(path[-1][1] < current_position[1] and path[-1][0] == current_position[0] and left):
						forward = 1
						left = 0
						current_position = path[-1]
						print("Path left to travel = {} ".format(path))
						print("Turn and Move Forward")
						print("Robot Position = {} ".format(current_position))
						path.pop()
				else:
					print("Home")
					RUNNING = 0
			


	except Exception as inst:
		motors.disengage()
		print("Robot Malfunction")
		#e = sys.exc_info()[0]
		#print(e)
		print(type(inst))    # the exception instance
		print(inst.args)     # arguments stored in .args
		print(inst)          # __str__ allows args to be printed directly,
							# but may be overridden in exception subclasses
		x = inst.args     # unpack args
		print('x =', x)


	finally:
		motors.disengage()
		IRsensor.closeI2C() #Close device
		pi.stop()
		sensorThread.join()
		visionThread.join()



if __name__ == '__main__':
	sensorThread = Thread(target=sensor)
	visionThread = Thread(target=camera)
	main()
	#map()
	

