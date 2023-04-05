from nodes import Nodes
import numpy as np
import rospy
import json
from configuration import *


if __name__ == "__main__":
    with open('mapper.json') as json_file:
        mapper = json.load(json_file)
    active_robots = list(mapper.keys())

    # Init phase
    print("start")   
    nodes = Nodes(active_robots)

    # if LOAD:
    #     last_saved_time = nodes.load_states(load_time=LOAD_TIME) +1
    # else:
    #     last_saved_time = 0

    # t = last_saved_time


    # Set Leds
    nodes.set_leds(green=0, blue=0, red=10)

    # TODO
    # reset the robot odom in the beginning
    
        
    for i in range(3):
        print("wait for odom response")
        nodes.move('still', step_size= 0.0, theta=0.)
        rospy.sleep(1)
    print("start reset")
    nodes.reset('theor')
    
    # Move Robots
    last_saved_time = 0
    step_size = 1.0
    theta = 0.0
    for t in range(last_saved_time, 10):
    
        print("t: ", t)
        nodes.store_data(t)
        nodes.move('move', step_size = step_size, theta = theta)
        step_size += 0.0
        theta += 0.0
        

        

    # Pull Odometry measure
    # nodes.print_position_measures()
    
    nodes.save_data(0)
    
    # Stop engine
    nodes.move('still', step_size= 0.0, theta=0.)
    
    print('done')


