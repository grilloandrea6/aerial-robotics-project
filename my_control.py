# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import utils
import random
from queue import PriorityQueue

# Global variables
on_ground = True
height_desired = 0.74
timer = None
startpos = None
timer_done = None

# Initial  map setup 

min_x, max_x = 0, 5.0  # meter
min_y, max_y = 0, 3.0  # meter
range_max = 2.0  # meter, maximum range of distance sensor
res_pos = 0.04  # meter # 0.05
conf = 0.1  # certainty given by each measurement 0.03
enlargement = 2  # Number of additional cells around the obstacle to mark as occupied # 1
t = 0  # only for plotting

map = np.zeros((int((max_x - min_x) / res_pos), int((max_y - min_y) / res_pos)))  # 0 = unknown, 1 = free, -1 = occupied


#####################MY CODE#####################
# Constants
forward_velocity = 0.20
CRUISE_HEIGHT = 0.5
landing_region_x = 3.6
command_threshold = 0.2 #meters, to avoid fast changes in the command

#Global variables
init = False
STATE_ON_GROUND = 0
STATE_FORWARD = 1
STATE_AVOID_FROM_RIGHT = 2
STATE_AVOID_FROM_LEFT = 3
STATE_MORE_RIGHT = 4
STATE_MORE_LEFT = 5
STATE_LOOK4PAD = 6
STATE_ASTAR_FOLLOW = 7
STATE_RETURN_FOLLOW = 8



# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py lines 296-323. 
# The "item" values that you can later use in the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "range_down": Downward range finder distance (Used instead of Global Z distance)
# "range_front": Front range finder distance
# "range_left": Leftward range finder distance 
# "range_right": Rightward range finder distance
# "range_back": Backward range finder distance
# "roll": Roll angle (rad)
# "pitch": Pitch angle (rad)
# "yaw": Yaw angle (rad)

# This is the main function where you will implement your control algorithm
def get_command(sensor_data,camera_data, dt):
    global init, control

    if not init:
        control = DroneControl()
        init = True

    # Open a window to display the camera image
    # NOTE: Displaying the camera image will slow down the simulation, this is just for testing
    #cv2.imshow('Camera Feed', camera_data)
    #cv2.waitKey(1)

    return control.get_command(sensor_data, dt)



# Class DroneControl----------------------------------------------------------------------------------------------------------------
class DroneControl:
    def __init__(self):
        self.startpos = None # Start position of the drone
        self.state = STATE_ON_GROUND # Initial state of the drone 
        self.old_control_command = [0.0, 0.0, 0.1, 0.0] # Initial control command

        self.x_goal = 0.0 # X coordinate of the goal when avoiding an obstacle to ramain at the same x coordinate
        self.y_end_obstacle = 0.0 # Y coordinate of the moment in which obstacle is avpided to count those 8 cm of extra displacement

        self.no_obstacle_threshold = 0.6 # Threshold to consider that there is no obstacle in front of the drone
        self.side_obstacle_threshold = 0.2 # Threshold to consider that there is an obstacle on the side of the drone


    def get_command(self, sensor_data, dt):
        if self.startpos is None:
            self.startpos = [sensor_data['x_global'], sensor_data['y_global']]

        # Get the current position of the drone
        current_pos = np.array([sensor_data['x_global'], sensor_data['y_global']])
        if self.state == STATE_ON_GROUND:
            if sensor_data['range_down'] > (0.9*CRUISE_HEIGHT - 0.1):
                self.state = STATE_FORWARD
                print("Change state: ", self.state)
            control_command = [0.0, 0.0, CRUISE_HEIGHT, 0.0]
            #control_command = [startpos[0], startpos[1], CRUISE_HEIGHT, 0.0] #For Hardware drone
            #control_command = command_threshold(old_control_command, control_command)
            #old_control_command = control_command
            return control_command
        
        if self.state == STATE_FORWARD:
            # If I find an obstacle with the front sensor I pass to the AVOID FROM RIGHT or LEFTAVOID FROM LEFT states depending on my porition wrt the center of the map
            if sensor_data['range_front'] < 0.3:
                if sensor_data['y_global'] > 1.5:
                    self.x_goal = sensor_data['x_global']
                    self.state = STATE_AVOID_FROM_RIGHT
                else:
                    self.x_goal = sensor_data['x_global']
                    self.state = STATE_AVOID_FROM_LEFT
                print("Change state: ", self.state)

            # If sensor_data['x_global'] is close to landing_region_x, go to look for pad state
            if sensor_data['x_global'] > landing_region_x - 0.1:
                self.state = STATE_LOOK4PAD
                print("Change state: ", self.state)
            control_command = [forward_velocity, 0.0, CRUISE_HEIGHT, 0.0]
            #control_command = [landing_region_x, 1.5, CRUISE_HEIGHT, 0.0] #For Hardware drone
            #control_command = command_threshold(old_control_command, control_command)
            #old_control_command = control_command
            return control_command
        
        if self.state == STATE_AVOID_FROM_RIGHT:
            '''I have to avoid the obstacle from the right side, which means reducing the y coordinate pointing at (x_goal, 0.1)'''

            # State changes to STATE_AVOID_FROM_LEFT if range_right is less than 0.3 --> I encountered an obstacle on the right so I have to avoid it from the left
            if sensor_data['range_right'] < self.side_obstacle_threshold:
                self.state = STATE_AVOID_FROM_LEFT
                self.x_goal = sensor_data['x_global']
                self.y_end_obstacle = sensor_data['y_global']
                print("Change state: ", self.state)

            # State changes if obstacle is avoided
            if sensor_data['range_front'] > self.no_obstacle_threshold:
                self.state = STATE_MORE_RIGHT
                self.x_goal = sensor_data['x_global']
                self.y_end_obstacle = sensor_data['y_global']
                print("Change state: ", self.state)
            control_command = [0.0, -forward_velocity, CRUISE_HEIGHT, 0.0]
            #control_command = [self.x_goal, 0.1, CRUISE_HEIGHT, 0.0] #For Hardware drone
            #control_command = command_threshold(old_control_command, control_command)
            #old_control_command = control_command
            return control_command
        
        if self.state == STATE_AVOID_FROM_LEFT:
            '''I have to avoid the obstacle from the left side, which means increasing the y coordinate pointing at (x_goal, 2.9)'''

            # State changes to STATE_AVOID_FROM_RIGHT if range_left is less than 0.3 --> I encountered an obstacle on the left so I have to avoid it from the right
            if sensor_data['range_left'] < self.side_obstacle_threshold:
                self.state = STATE_AVOID_FROM_RIGHT
                self.x_goal = sensor_data['x_global']
                self.y_end_obstacle = sensor_data['y_global']
                print("Change state: ", self.state)

            # State changes if obstacle is avoided
            if sensor_data['range_front'] > self.no_obstacle_threshold:
                self.state = STATE_MORE_LEFT
                self.x_goal = sensor_data['x_global']
                self.y_end_obstacle = sensor_data['y_global']
                print("Change state: ", self.state)
            control_command = [0.0, forward_velocity, CRUISE_HEIGHT, 0.0]
            #control_command = [self.x_goal, 2.9, CRUISE_HEIGHT, 0.0] #For Hardware drone
            #control_command = command_threshold(old_control_command, control_command)
            #old_control_command = control_command
            return control_command
        
        if self.state == STATE_MORE_RIGHT:
            '''Go right for 10 cm more before passing back to going forward'''
            # If you arrived at the y_end_obstacle - 0.01, you can go forward again
            if sensor_data['y_global'] <= self.y_end_obstacle - 0.1:
                self.state = STATE_FORWARD
                print("Change state: ", self.state)
            control_command = [0.0, -forward_velocity, CRUISE_HEIGHT, 0.0]
            #control_command = [self.x_goal, 0.1, CRUISE_HEIGHT, 0.0] #For Hardware drone
            #control_command = command_threshold(old_control_command, control_command)
            #old_control_command = control_command
            return control_command
        
        if self.state == STATE_MORE_LEFT:
            '''Go left for 10 cm more before passing back to going forward'''
            # If you arrived at the y_end_obstacle + 0.01, you can go forward again
            if sensor_data['y_global'] >= self.y_end_obstacle + 0.1:
                self.state = STATE_FORWARD
                print("Change state: ", self.state)
            control_command = [0.0, forward_velocity, CRUISE_HEIGHT, 0.0]
            #control_command = [self.x_goal, 2.9, CRUISE_HEIGHT, 0.0] #For Hardware drone
            #control_command = command_threshold(old_control_command, control_command)
            #old_control_command = control_command
            return control_command
        
        if self.state == STATE_LOOK4PAD:
            control_command = [0.0, 0.0, CRUISE_HEIGHT, 0.0]
            return control_command
        

        
    # UTILS for DroneControl----------------------------------------------------------------------------------------------------------------

    def command_threshold(self, old_control_command, new_control_command, threshold = command_threshold, increment = command_threshold):
        '''Check if the new command is higher than the threshold: in this case return the old command plus an increment, otherwise return the new command. Do it for every coordinate, x, y anz z'''
        new_control_command = np.array(new_control_command)
        old_control_command = np.array(old_control_command)
        for i in range(3):
            if new_control_command[i] - old_control_command[i] > threshold:
                new_control_command[i] = old_control_command[i] + increment
            elif old_control_command[i] - new_control_command[i] < - threshold:
                new_control_command[i] = old_control_command[i] - increment
        return new_control_command

    
    
    