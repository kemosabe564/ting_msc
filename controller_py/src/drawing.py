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
T = 300

offs = 3

if __name__ == "__main__":

    robot_history = dict.fromkeys(['0', '1', '2'], dict())
    
    key_set = ['orien', 'pos_x', 'pos_y', 'estimation_phi', 'estimation_x', 'estimation_y', 'cam_phi', 'cam_x', 'cam_y', 'x', 'y', 'phi', 'P_k_odo', 'P_k_cam', 'OWA_w1', 'OWA_w2', 'accelx', 'accelx_lowpass', 'accelxPos', 'accelyPos']
    
    with open('./data/saved_data_t0_RUN_test_cas.p', 'rb') as fp:
        if P == 3:
            b = pickle.load(fp)
        elif P == 2:
            b = pickle.load(fp, encoding='iso-8859-1')
    # print(b)
    for i in range(T):
        if(i == 0):
            for j in range(robot_N):
                robot_history[str(j)] = copy.deepcopy(b[i][str(j)])
                robot_history[str(j)][0] = dict.fromkeys(key_set, dict())
                
                for k in range(len(key_set)):
                    robot_history[str(j)][0][key_set[k]] = copy.deepcopy(b[i][str(j)][key_set[k]])
                    del robot_history[str(j)][key_set[k]]
                # robot_history[str(j)][0]['pos_x'] = copy.deepcopy(b[i][str(j)]['pos_x'])
                # robot_history[str(j)][0]['pos_y'] = copy.deepcopy(b[i][str(j)]['pos_y'])
                # robot_history[str(j)][0]['orien'] = copy.deepcopy(b[i][str(j)]['orien'])

                # del robot_history[str(j)]['pos_x']
                # del robot_history[str(j)]['pos_y']
                # del robot_history[str(j)]['orien']

        else:
            for j in range(robot_N):
                robot_history[str(j)][i] = copy.deepcopy(b[i][str(j)])
    
    t = np.arange(T) * 0.1

    # a = pd.DataFrame(robot_history['2']).T
    # print('2')
    # print(a)
    plt.figure(figsize = (8, 6), dpi = 80)

    for i in range(robot_N):
        a = pd.DataFrame(robot_history[str(i)]).T
    
        plt.plot(a['estimation_x'][offs:], a['estimation_y'][offs:], linestyle='--', marker='o', label='line with marker')
    
        
    plt.legend(['Robot 1', 'Robot 2', 'Robot 3'])
    plt.xlabel("x position/m")
    plt.ylabel("y position/m")
    for i in range(robot_N):
        a = pd.DataFrame(robot_history[str(i)]).T
        plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=12, markeredgewidth = 4)
        plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=12, markeredgewidth = 4)
    plt.title('Robots Tracking Result')
    
    
    
    a = pd.DataFrame(robot_history['0']).T
    print(a)
    # a = pd.DataFrame(robot_history['0']).T
    # print('0')
    # print(a)
    # print(a['pos_x'])

    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(a['pos_x'][offs:], a['pos_y'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.title('Odometry')
    
    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(a['cam_x'][offs:], a['cam_y'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.title('Camera')
    
    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(a['estimation_x'][offs:], a['estimation_y'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.title('Estimation')
    
    
    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(t, a['OWA_w1'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.title('OWA_w1')
    
    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(t, a['OWA_w2'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.title('OWA_w2')

    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(t, a['OWA_w3'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.title('OWA_w3')


    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(a['accelxPos'][offs:], a['accelyPos'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.title('accel')
    

    
    
    plt.figure(figsize = (8, 6), dpi = 80)
    plt.plot(a['pos_x'][offs:], a['pos_y'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.plot(a['cam_x'][offs:], a['cam_y'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.plot(a['accelxPos'][offs:], a['accelyPos'][offs:], linestyle='--', marker='o', label='line with marker')    
    plt.plot(a['estimation_x'][offs:], a['estimation_y'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.xlabel("x position/m")
    plt.ylabel("y position/m")
    plt.title('Sensor Data Comparison for Single Robot')
    
    plt.legend(['Odometry', 'Camera', 'Estimation'])
    plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=12, markeredgewidth = 4)
    plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=12, markeredgewidth = 4)


    
    plt.show()
    

    
    
    
    
    
    