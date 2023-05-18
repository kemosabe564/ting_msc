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
offset = 5
T = 1600

if __name__ == "__main__":

    robot_history = dict.fromkeys(['0', '1', '2'], dict())

    key_set = ['orien', 'pos_x', 'pos_y', 'estimation_phi', 'estimation_x', 'estimation_y', 'cam_phi', 'cam_x', 'cam_y',
               'x', 'y', 'phi', 'P_k_odo', 'P_k_cam', 'OWA_w1', 'OWA_w2'] # 'P_k_odo', 'P_k_cam' na phi

    with open('./data/data_ch7/petc_3_line_line_video.p', 'rb') as fp: #6
    #with open('./data/saved_data_t0_RUN6.p', 'rb') as fp:  # 6
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
                # robot_history[str(j)][0]['pos_x'] = copy.deepcopy(b[i][str(j)]['pos_x'])
                # robot_history[str(j)][0]['pos_y'] = copy.deepcopy(b[i][str(j)]['pos_y'])
                # robot_history[str(j)][0]['orien'] = copy.deepcopy(b[i][str(j)]['orien'])

                # del robot_history[str(j)]['pos_x']
                # del robot_history[str(j)]['pos_y']
                # del robot_history[str(j)]['orien']

        else:
            for j in range(robot_N):
                robot_history[str(j)][i] = copy.deepcopy(b[i][str(j)])

    # print(b[0])
    # print(b[0][str(1)])

    # print(robot_history['0'])
    # print(robot_history['2'][1])

    t = np.arange(T)*0.05

    # a = pd.DataFrame(robot_history['2']).T
    # print('2')
    # print(a)
    a = pd.DataFrame(robot_history['0']).T
    b = pd.DataFrame(robot_history['1']).T
    c = pd.DataFrame(robot_history['2']).T
    #d = pd.DataFrame(robot_history['3']).T
    print('1')
    print(a)
    # a = pd.DataFrame(robot_history['0']).T
    # print('0')
    # print(a)
    # print(a['pos_x'])

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '.', color='#0072BD')
    plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '.', color='#7E2F8E')
    plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '.', color='#77AC30')
    #plt.plot(d['pos_x'][offset:], d['pos_y'][offset:], '.', color='#EDB120')
    plt.plot(a['pos_x'][offset], a['pos_y'][offset], '*', color='g', markersize=15)
    plt.plot(a['pos_x'][T - 1], a['pos_y'][T - 1], 'X', color='r', markersize=10)
    plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '-', color='#0072BD', alpha=0.2)
    plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '-', color='#7E2F8E', alpha=0.2)
    plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '-', color='#77AC30', alpha=0.2)
    #plt.plot(d['pos_x'][offset:], d['pos_y'][offset:], '-', color='#EDB120', alpha=0.2)
    plt.plot(b['pos_x'][offset], b['pos_y'][offset], '*', color='g', markersize=15)
    plt.plot(c['pos_x'][offset], c['pos_y'][offset], '*', color='g', markersize=15)
    #plt.plot(d['pos_x'][offset], d['pos_y'][offset], '*', color='g', markersize=15)
    plt.plot(b['pos_x'][T - 1], b['pos_y'][T - 1], 'X', color='r', markersize=10)
    plt.plot(c['pos_x'][T - 1], c['pos_y'][T - 1], 'X', color='r', markersize=10)
    #plt.plot(d['pos_x'][T - 1], d['pos_y'][T - 1], 'X', color='r', markersize=10)
    plt.legend(["Robot 0", "Robot 1", "Robot 2", "Start", "Finish"])
    plt.xlabel("x-coordinate [m]")
    plt.ylabel("y-coordinate [m]")
    # plt.xlim([0.5,1.5])
    # plt.ylim([0.6,1.5])
    plt.title('Odometry')

    # plt.figure(figsize = (8, 6), dpi = 80)
    # plt.plot(a['x'], a['y'], '.')

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '.', color='#0072BD')
    plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '.', color='#7E2F8E')
    plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '.', color='#77AC30')
    #plt.plot(d['cam_x'][offset:], d['cam_y'][offset:], '.', color='#EDB120')
    plt.plot(a['cam_x'][offset], a['cam_y'][offset], '*', color='g', markersize=15)
    plt.plot(a['cam_x'][T-1], a['cam_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '-', color='#0072BD', alpha=0.2)
    plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '-', color='#7E2F8E', alpha=0.2)
    plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '-', color='#77AC30', alpha=0.2)
    #plt.plot(d['cam_x'][offset:], d['cam_y'][offset:], '-', color='#EDB120', alpha=0.2)
    plt.plot(b['cam_x'][offset], b['cam_y'][offset], '*', color='g', markersize=15)
    plt.plot(c['cam_x'][offset], c['cam_y'][offset], '*', color='g', markersize=15)
    #plt.plot(d['cam_x'][offset], d['cam_y'][offset], '*', color='g', markersize=15)
    # plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], '*', color='r', markersize=10)
    plt.plot(b['cam_x'][T-1], b['cam_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(c['cam_x'][T-1], c['cam_y'][T-1], 'X', color='r', markersize=10)
    #plt.plot(d['cam_x'][T-1], d['cam_y'][T-1], 'X', color='r', markersize=10)
    plt.legend(["Robot 0", "Robot 1", "Robot 2", "Start", "Finish"])
    plt.xlabel("x-coordinate [m]")
    plt.ylabel("y-coordinate [m]")
    #plt.xlim([0,2])
    #plt.ylim([0,2])
    plt.title('Camera')

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '.', color='#0072BD')
    plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '.', color='#7E2F8E')
    plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '.', color='#77AC30')
    #plt.plot(d['estimation_x'][offset:], d['estimation_y'][offset:], '.', color='#EDB120')
    plt.plot(a['estimation_x'][offset], a['estimation_y'][offset], '*', color='g', markersize=15)
    plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '-', color='#0072BD', alpha=0.2)
    plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '-', color='#7E2F8E', alpha=0.2)
    plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '-', color='#77AC30', alpha=0.2)
    #plt.plot(d['estimation_x'][offset:], d['estimation_y'][offset:], '-', color='#EDB120', alpha=0.2)
    plt.plot(b['estimation_x'][offset], b['estimation_y'][offset], '*', color='g', markersize=15)
    plt.plot(c['estimation_x'][offset], c['estimation_y'][offset], '*', color='g', markersize=15)
    #plt.plot(d['estimation_x'][offset], d['estimation_y'][offset], '*', color='g', markersize=15)
    # plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], '*', color='r', markersize=10)
    plt.plot(b['estimation_x'][T-1], b['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(c['estimation_x'][T-1], c['estimation_y'][T-1], 'X', color='r', markersize=10)
    #plt.plot(d['estimation_x'][T-1], d['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.legend(["Robot 0", "Robot 1", "Robot 2", "Start", "Finish"])
    plt.xlabel("x-coordinate [m]")
    plt.ylabel("y-coordinate [m]")
    #plt.xlim([0,2])
    #plt.ylim([0,2])
    # plt.title('Estimation')
    plt.title('Trajectory of phase algorithm')

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '.')
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '-k', alpha=0.2)
    plt.plot(a['pos_x'][offset], a['pos_y'][offset], 'x', color='r', markersize=10)
    plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], 'x', color='g', markersize=10)
    plt.xlabel("time")
    plt.ylabel("speed")
    #plt.xlim([0,2])
    #plt.ylim([0,2])
    plt.title('Estimation')

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '.')
    plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '.')
    plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '.')
    plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '-k', alpha=0.2)
    plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '-k', alpha=0.2)
    plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '-k', alpha=0.2)
    plt.plot(b['pos_x'][offset], b['pos_y'][offset], 'x', color='r', markersize=10)
    plt.plot(b['pos_x'][T-1], b['pos_y'][T-1], 'x', color='g', markersize=10)
    plt.legend(["odom", "cam", "estimation"])
    plt.xlabel("x-coordinate [m]")
    plt.ylabel("y-coordinate [m]")
    #plt.xlim([0,2])
    #plt.ylim([0,2])
    plt.title('Merged robot 1')


    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '.')
    plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '.')
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '.')
    plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '-k', alpha=0.2)
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '-k', alpha=0.2)
    plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '-k', alpha=0.2)
    plt.plot(a['pos_x'][offset], a['pos_y'][offset], 'x', color='r', markersize=10)
    plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], 'x', color='g', markersize=10)
    plt.legend(["odom", "cam", "estimation"])
    plt.xlabel("x-coordinate [m]")
    plt.ylabel("y-coordinate [m]")
    #plt.xlim([0,2])
    #plt.ylim([0,2])
    plt.title('Merged robot 0')

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '.')
    plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '.')
    plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '.')
    plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '-k', alpha=0.2)
    plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '-k', alpha=0.2)
    plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '-k', alpha=0.2)
    plt.plot(c['pos_x'][offset], c['pos_y'][offset], 'x', color='r', markersize=10)
    plt.plot(c['pos_x'][T-1], c['pos_y'][T-1], 'x', color='g', markersize=10)
    plt.legend(["odom", "cam", "estimation"])
    plt.xlabel("x-coordinate [m]")
    plt.ylabel("y-coordinate [m]")
    #plt.xlim([0,2])
    #plt.ylim([0,2])
    plt.title('Merged robot 2')

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(t[offset-2:], b['pos_y'][offset:], '.')
    plt.plot(t[offset-2:], b['cam_y'][offset:], '.')
    plt.plot(t[offset-2:], b['estimation_y'][offset:], '.')
    plt.plot(t[offset-2:], b['pos_y'][offset:], '-k', alpha=0.2)
    plt.plot(t[offset-2:], b['estimation_y'][offset:], '-k', alpha=0.2)
    plt.plot(t[offset-2:], b['cam_y'][offset:], '-k', alpha=0.2)
    plt.legend(["odom", "cam", "estimation"])
    plt.xlabel("time")
    plt.ylabel("y-coordinate [m]")
    #plt.xlim([0,2])
    #plt.ylim([0,2])
    plt.title('Time plot robot 1')


    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(t, c['cam_x'][2:], '.')
    # plt.plot(t, c['cam_x'][2:], '-k', alpha=0.2)
    # plt.xlabel("time")
    # plt.ylabel("x-coordinate [m]")
    # #plt.xlim([0,2])
    # #plt.ylim([0,2])
    # plt.title('Camera - x position')
    #
    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(t, c['cam_y'][2:], '.')
    # plt.plot(t, c['cam_y'][2:], '-k', alpha=0.2)
    # plt.xlabel("time")
    # plt.ylabel("y-coordinate [m]")
    # #plt.xlim([0,2])
    # #plt.ylim([0,2])
    # plt.title('Camera - y position')

    plt.show()

    # plt.figure(figsize = (8, 6), dpi = 80)
    # plt.plot(a['estimation_x'], a['estimation_y'], '.')
    # plt.title('estimation')






