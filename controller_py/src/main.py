from nodes import Nodes
import numpy as np

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

    # Move Robots
    last_saved_time = 0
    step_size = 1.0
    theta = 0.0
    for t in range(last_saved_time, 20):
    
        print("t: ", t)
        nodes.move(step_size = step_size, theta=theta)
        step_size += 0.1
        theta += 0.0
        
        
        nodes.store_data(t)

    # Pull Odometry measure
    # nodes.print_position_measures()
    
    nodes.save_data(0)
    
    # Stop engine
    nodes.move(step_size= 0.0, theta=0.)
    
    print('done')


