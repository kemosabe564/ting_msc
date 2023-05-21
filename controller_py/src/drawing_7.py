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

robot_N = 4
offset = 5
T = 1000

figsize_x = 12
figsize_y = 5
dpi_number = 100

label_size = 16
title_size = 16
ticks_size = 12
legend_size = 12
marker_size = 4
f = './data/data_ch7/phase_4_line_circle_video_bestone.p'

if __name__ == "__main__":

    robot_history = dict.fromkeys(['0', '1', '2', '3'], dict())

    key_set = ['orien', 'pos_x', 'pos_y', 'estimation_phi', 'estimation_x', 'estimation_y', 'cam_phi', 'cam_x', 'cam_y',
               'x', 'y', 'phi', 'P_k_odo', 'P_k_cam', 'OWA_w1', 'OWA_w2'] # 'P_k_odo', 'P_k_cam' na phi
    # key_set = ['orien', 'pos_x', 'pos_y', 'estimation_phi', 'estimation_x', 'estimation_y', 'cam_phi', 'cam_x', 'cam_y',
    #            'x', 'y', 'phi'] # 'P_k_odo', 'P_k_cam' na phi

    with open(f, 'rb') as fp:
        if P == 3:
            b = pickle.load(fp)
        elif P == 2:
            b = pickle.load(fp, encoding='iso-8859-1')
    # print(b)
    for i in range(T):
        if (i == 0):
            for j in range(robot_N):
                robot_history[str(j)] = copy.deepcopy(b[i][str(j)])
                robot_history[str(j)][0] = dict.fromkeys(key_set, dict())

                for k in range(len(key_set)):
                    robot_history[str(j)][0][key_set[k]] = copy.deepcopy(b[i][str(j)][key_set[k]])
                    del robot_history[str(j)][key_set[k]]

        else:
            for j in range(robot_N):
                robot_history[str(j)][i] = copy.deepcopy(b[i][str(j)])

    t = np.arange(T)*0.05

    # a = pd.DataFrame(robot_history['2']).T
    # print('2')
    # print(a)
    a = pd.DataFrame(robot_history['0']).T
    b = pd.DataFrame(robot_history['1']).T
    c = pd.DataFrame(robot_history['2']).T
    d = pd.DataFrame(robot_history['3']).T
    # print(d)

    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size)
    plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size)
    plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size)
    plt.plot(d['estimation_x'][offset:], d['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size)

    plt.plot(a['estimation_x'][offset], a['estimation_y'][offset], '*', color='g', markersize=15)
    plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(b['estimation_x'][offset], b['estimation_y'][offset], '*', color='g', markersize=15)
    plt.plot(c['estimation_x'][offset], c['estimation_y'][offset], '*', color='g', markersize=15)    
    plt.plot(d['estimation_x'][offset], d['estimation_y'][offset], '*', color='g', markersize=15)    

    plt.plot(b['estimation_x'][T-1], b['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(c['estimation_x'][T-1], c['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(d['estimation_x'][T-1], d['estimation_y'][T-1], 'X', color='r', markersize=10)

    # plt.legend(["Robot 1", "Robot 2", "Robot 3", "Start", "Finish"], fontsize = legend_size)
    plt.legend(["Robot 1", "Robot 2", "Robot 3", "Robot 4", "Start", "Finish"], fontsize = legend_size)

    plt.xlabel("x position/m", fontsize = label_size)
    plt.ylabel("y position/m", fontsize = label_size)
    plt.yticks(fontsize = ticks_size)

    plt.xticks(fontsize = ticks_size)
    plt.grid()
    plt.title('Trajectory of swarm', fontsize = title_size)

    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    # plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '-k', alpha=0.2)
    # plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '-k', alpha=0.2)
    
    plt.plot(a['pos_x'][offset], a['pos_y'][offset], '*', color='b', markersize=15)
    plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], 'X', color='r', markersize=10)    
    plt.legend(['Odometry', 'Camera', 'Estimation'], fontsize = legend_size)
    plt.xlabel("x position/m", fontsize = label_size)
    plt.ylabel("y position/m", fontsize = label_size)
    plt.grid()
    plt.title('Robot 1', fontsize = title_size)



    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    # plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '-k', alpha=0.2)
    # plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '-k', alpha=0.2)
    
    plt.plot(b['pos_x'][offset], b['pos_y'][offset], '*', color='b', markersize=15)
    plt.plot(b['pos_x'][T-1], b['pos_y'][T-1], 'X', color='r', markersize=10) 
    plt.legend(['Odometry', 'Camera', 'Estimation'], fontsize = legend_size)
    plt.xlabel("x position/m", fontsize = label_size)
    plt.ylabel("y position/m", fontsize = label_size)
    plt.grid()
    plt.title('Robot 2', fontsize = title_size)




    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    # plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '-k', alpha=0.2)
    # plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '-k', alpha=0.2)
    
    plt.plot(c['pos_x'][offset], c['pos_y'][offset], '*', color='b', markersize=15)
    plt.plot(c['pos_x'][T-1], c['pos_y'][T-1], 'X', color='r', markersize=10) 
    plt.legend(['Odometry', 'Camera', 'Estimation'], fontsize = legend_size)
    plt.xlabel("x position/m", fontsize = label_size)
    plt.ylabel("y position/m", fontsize = label_size)
    plt.grid()
    plt.title('Robot 3', fontsize = title_size)
    
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    plt.plot(d['pos_x'][offset:], d['pos_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    plt.plot(d['cam_x'][offset:], d['cam_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    plt.plot(d['estimation_x'][offset:], d['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size - 1)
    # plt.plot(d['pos_x'][offset:], d['pos_y'][offset:], '-k', alpha=0.2)
    # plt.plot(d['estimation_x'][offset:], d['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(d['cam_x'][offset:], d['cam_y'][offset:], '-k', alpha=0.2)
    
    plt.plot(d['pos_x'][offset], d['pos_y'][offset], '*', color='b', markersize=15)
    plt.plot(d['pos_x'][T-1], d['pos_y'][T-1], 'X', color='r', markersize=10) 
    plt.legend(['Odometry', 'Camera', 'Estimation'], fontsize = legend_size)
    plt.xlabel("x position/m", fontsize = label_size)
    plt.ylabel("y position/m", fontsize = label_size)
    plt.grid()
    plt.title('Robot 4', fontsize = title_size)

    if (f == './data/data_ch7/phase_3_line_no_estimation.p'):
        
        plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
        plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], linestyle='--', marker='o', markersize = marker_size)
        plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], linestyle='--', marker='o', markersize = marker_size)
        plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], linestyle='--', marker='o', markersize = marker_size)
        plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size)
        plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size)
        plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], linestyle='--', marker='o', markersize = marker_size)

        plt.plot(a['estimation_x'][offset], a['estimation_y'][offset], '*', color='g', markersize=15)
        plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], 'X', color='r', markersize=10)
        plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], 'X', color='b', markersize=10)
        plt.plot(b['estimation_x'][offset], b['estimation_y'][offset], '*', color='g', markersize=15)
        plt.plot(c['estimation_x'][offset], c['estimation_y'][offset], '*', color='g', markersize=15)    
        plt.plot(b['estimation_x'][T-1], b['estimation_y'][T-1], 'X', color='r', markersize=10)
        plt.plot(c['estimation_x'][T-1], c['estimation_y'][T-1], 'X', color='r', markersize=10)
        plt.legend(["Robot 1 (Odometry)", "Robot 2 (Odometry)", "Robot 3 (Odometry)","Robot 1 (real)", "Robot 2 (real)", "Robot 3 (real)", "Start", "Finish (real)", "Finish (Odometry)"], fontsize = legend_size)
        
        

        # plt.plot(a['pos_x'][offset], a['pos_y'][offset], '*', color='g', markersize=15)
        
        # plt.plot(b['pos_x'][offset], b['pos_y'][offset], '*', color='g', markersize=15)
        # plt.plot(c['pos_x'][offset], c['pos_y'][offset], '*', color='g', markersize=15)    
        plt.plot(b['pos_x'][T-1], b['pos_y'][T-1], 'X', color='b', markersize=10)
        plt.plot(c['pos_x'][T-1], c['pos_y'][T-1], 'X', color='b', markersize=10)
        
        
        
        
        plt.xlabel("x position/m", fontsize = label_size)
        plt.ylabel("y position/m", fontsize = label_size)
        plt.yticks(fontsize = ticks_size)

        plt.xticks(fontsize = ticks_size)
        plt.grid()
        plt.title('Trajectory of swarm', fontsize = title_size)



    plt.show()







