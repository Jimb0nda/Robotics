import motors
import odometry
import robot_main
import random
import time

# Moves robot forward while seaching for the end goal
def searching():
    F1 = motors.Forward(motors.speed)
    F1.forward()

# If obstacle is detected, randomly turns left or right to avoid
def avoidance():
    turn = random.randint(1,2)
    if turn == 1:
        motors.left(90)
    elif turn == 2:
        motors.right(90)
    
    
    
