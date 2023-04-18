import numpy as np
import functions
import time
import json
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import copy
import pickle

import pandas as pd 

P = 3

robot_N = 3
T = 30000

if __name__ == "__main__":

    robot_history = dict.fromkeys(['0', '1', '2'], dict())
    
    key_set = ['pos_x', 'pos_y', 'orien', 'cam_x', 'cam_phi', 'cam_y', 'odom_timer', 'cam_timer']
    
    with open('./data/all_data_t0_RUN1.p', 'rb') as fp:
        if P == 3:
            b = pickle.load(fp)
        elif P == 2:
            b = pickle.load(fp, encoding='iso-8859-1')
    # print(b)
    print(b[1])
    print(robot_history[str(0)])
    for i in range(T):
        if(i == 1):
            for j in range(robot_N):
                robot_history[str(j)] = copy.deepcopy(b[i + 1][(j)])
                robot_history[str(j)][0] = dict.fromkeys(key_set, dict())
                
                for k in range(len(key_set)):
                    robot_history[str(j)][0][key_set[k]] = copy.deepcopy(b[i][(j)][key_set[k]])
                    del robot_history[str(j)][key_set[k]]
                # robot_history[str(j)][0]['pos_x'] = copy.deepcopy(b[i][str(j)]['pos_x'])
                # robot_history[str(j)][0]['pos_y'] = copy.deepcopy(b[i][str(j)]['pos_y'])
                # robot_history[str(j)][0]['orien'] = copy.deepcopy(b[i][str(j)]['orien'])

                # del robot_history[str(j)]['pos_x']
                # del robot_history[str(j)]['pos_y']
                # del robot_history[str(j)]['orien']

        else:
            for j in range(robot_N):
                robot_history[str(j)][i] = copy.deepcopy(b[i + 1][(j)])


    # print(b[0])
    # print(b[0][str(1)])

    # print(robot_history['0'])
    # print(robot_history['2'][1])
    
    t = np.arange(T)
    
    # a = pd.DataFrame(robot_history['2']).T
    # print('2')
    # print(a)
    a = pd.DataFrame(robot_history['0']).T
    print('1')
    print(a)
    # a = pd.DataFrame(robot_history['0']).T
    # print('0')
    # print(a)
    # print(a['pos_x'])

    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(a['pos_x'], a['pos_y'], '.')
    plt.title('odo')
    
    # plt.figure(figsize = (8, 6), dpi = 80)
    # plt.plot(a['x'], a['y'], '.')
    
    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(a['cam_x'], a['cam_y'], '.')
    plt.title('camera')
    
    plt.show()
    
    
    
    
    
    