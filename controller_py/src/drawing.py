import numpy as np
import functions
import time
import json
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import copy
import pickle
import matplotlib.animation as animation

import pandas as pd 

P = 3
figsize_x = 12
figsize_y = 5
dpi_number = 100

label_size = 16
title_size = 16
ticks_size = 12
legend_size = 12

robot_N = 3
T = 300

offs = 10

# f = './data/saved_data_t10_RUN_test.p'
# OWA_noreset_low_f_mr
f = 'controller_py/src/data/data_ch6/saved_data_t0_RUN_test_OWA_noreset_low_mr.p'
# f = 'controller_py/src/data/data_ch7/etc_4_HC_circle_video.p'


# f = './data/data/saved_data_t0_RUN_noise_acc_still.p'


if __name__ == "__main__":

    robot_history = dict.fromkeys(['0', '1', '2'], dict())
    
    key_set = ['pos_x', 'pos_y', 'orien', 'estimation_x', 'estimation_y', 'estimation_phi', 'cam_x', 'cam_y', 'cam_phi', 'x', 'y', 'phi', 'P_k_odo', 'P_k_cam', 'odom_timer', 'cam_timer', 'OWA_w1', 'OWA_w2', 'OWA_w3', 'accelx', 'accelx_lowpass', 'accelxPos', 'accelyPos', 'full_camera']
    key_set = ['pos_x', 'pos_y', 'orien', 'estimation_x', 'estimation_y', 'estimation_phi', 'cam_x', 'cam_y', 'cam_phi', 'x', 'y', 'phi', 'P_k_odo', 'P_k_cam', 'odom_timer', 'cam_timer', 'OWA_w1', 'OWA_w2', 'OWA_w3', 'accelx', 'accelx_lowpass', 'accelxPos', 'accelyPos']
    key_set = ['pos_x', 'pos_y', 'orien', 'estimation_x', 'estimation_y', 'estimation_phi', 'cam_x', 'cam_y', 'cam_phi', 'x', 'y', 'phi', 'P_k_odo', 'P_k_cam', 'odom_timer', 'cam_timer', 'OWA_w1', 'OWA_w2']
    
    with open(f, 'rb') as fp:
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
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)

    for i in range(robot_N):
        a = pd.DataFrame(robot_history[str(i)]).T
    
        plt.plot(a['estimation_x'][offs:], a['estimation_y'][offs:], linestyle='--', marker='o', label='line with marker')
    
        
    plt.legend(['Robot 1', 'Robot 2', 'Robot 3'], fontsize = legend_size)
    plt.xlabel("x position/m", fontsize = label_size)
    plt.ylabel("y position/m", fontsize = label_size)
    for i in range(robot_N):
        a = pd.DataFrame(robot_history[str(i)]).T
        plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=12, markeredgewidth = 4)
        plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=12, markeredgewidth = 4)
    plt.title('Robots Tracking Result', fontsize = title_size)
    plt.yticks(fontsize = ticks_size)
    plt.xticks(fontsize = ticks_size)
    plt.grid()
    
    
    a = pd.DataFrame(robot_history['2']).T
    # print(a)
    # # a = pd.DataFrame(robot_history['0']).T
    # # print('0')
    # # print(a)
    # # print(a['pos_x'])

    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(a['pos_x'][offs:], a['pos_y'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.title('Odometry')
    
    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(a['cam_x'][offs:], a['cam_y'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.title('Camera')
    
    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(a['estimation_x'][offs:], a['estimation_y'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.title('Estimation')
    
    
    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(t, a['OWA_w1'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.title('OWA_w1')
    
    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(t, a['OWA_w2'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.title('OWA_w2')

    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(t, a['OWA_w3'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.title('OWA_w3')
    
    


    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(a['accelxPos'][offs:], a['accelyPos'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.title('accel')
    

    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(t, a['OWA_w1'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.plot(t, a['OWA_w2'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.xlabel("time/s", fontsize = label_size)
    # plt.ylabel("weight", fontsize = label_size)
    # plt.title('Weight Comparison', fontsize = title_size)
    # plt.legend(['Odometry', 'Camera'], fontsize = legend_size)
    # plt.yticks(fontsize = ticks_size)
    # plt.xticks(fontsize = ticks_size)
    # plt.grid()

    
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    plt.plot(a['pos_x'][offs:], a['pos_y'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.plot(a['cam_x'][offs:], a['cam_y'][offs:], linestyle='--', marker='o', label='line with marker')
    # plt.plot(a['accelxPos'][offs:], a['accelyPos'][offs:], linestyle='--', marker='o', label='line with marker')    
    # plt.plot(a['estimation_x'][offs:], a['estimation_y'][offs:], linestyle='--', marker='o', label='line with marker')
    plt.xlabel("x position/m", fontsize = label_size)
    plt.ylabel("y position/m", fontsize = label_size)
    plt.title('Sensor Data Comparison for Single Robot', fontsize = title_size)
    
    plt.yticks(fontsize = ticks_size)

    plt.xticks(fontsize = ticks_size)

    # plt.legend(['Odometry', 'Camera', 'Accelerometer', 'Estimation'], fontsize = legend_size)
    plt.legend(['Odometry', 'Camera'], fontsize = legend_size)
    # plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=12, markeredgewidth = 4)
    # plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=12, markeredgewidth = 4)
    plt.grid()


    
    # # plt.show()
    
    
    # # print(a['estimation_x'][offs:T-100:2])
    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)

    # plt.plot(a['estimation_x'][offs:T:3], a['estimation_y'][offs:T:3], linestyle='--', marker='o', label='line with marker')
    # plt.plot(a['cam_x'][offs:T:3], a['cam_y'][offs:T:3], linestyle='--', marker='o', label='line with marker')
    # plt.title('Sensor asdasdasdasd', fontsize = title_size)
    # asd = sum((a['estimation_x'][offs:T:3] - a['cam_x'][offs:T:3]) ** 2 + (a['estimation_y'][offs:T:3] - a['cam_y'][offs:T:3]) ** 2)
    
    # print(a['estimation_x'][offs:T:3])
    # print(a['cam_x'][offs:T:3])
    # print(asd/100)
    # plt.show()
    
    GIF_single = 0
    
    if(GIF_single):
        
        name_list_x = ['estimation_x', 'pos_x', 'cam_x', 'accelxPos']
        name_list_y = ['estimation_y', 'pos_y', 'cam_y', 'accelyPos']
        
        fig = plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
        plt.grid()
        ims = []
    
        temp_x = []
        temp_y = []
        for i in range(4):
            temp_x.append([])
            temp_y.append([])
        # print(temp_x)    
        for i in range(T):
        
            
            for j in range(4):
                # a = b[j]
                if(len(temp_x[j]) > 20):
                    temp_x[j].pop(0)
                    temp_y[j].pop(0)
                # print(a)
                temp_x[j].append(a[name_list_x[j]][i])
                temp_y[j].append(a[name_list_y[j]][i])
            # for j in range(2):
                
            #     im, = plt.plot(temp_x[j], temp_y[j], linestyle='--', marker='o', label='line with marker', markersize = 4)
            #     ims.append([im])
            im1, = plt.plot(temp_x[0], temp_y[0], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:blue')
            im2, = plt.plot(temp_x[1], temp_y[1], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:orange')
            im3, = plt.plot(temp_x[2], temp_y[2], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:green')
            im4, = plt.plot(temp_x[3], temp_y[3], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:red')
            # im5, = plt.plot(temp_x[4], temp_y[4], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:purple')
            # im6, = plt.plot(temp_x[5], temp_y[5], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:brown')
            # im7, = plt.plot(temp_x[6], temp_y[6], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:pink')
            # im8, = plt.plot(temp_x[7], temp_y[7], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:gray')
            
            
            # print(type(im))
            # print(type(plt.gcf()))
            # im = plt.gcf()
            plt.xlabel("x position/m", fontsize = label_size)
            plt.ylabel("y position/m", fontsize = label_size)
            plt.xlim([0, 0.6])
            plt.ylim([0.5, 1.1])
            plt.title('Robots Tracking Result', fontsize = title_size)
            plt.yticks(fontsize = ticks_size)
            plt.xticks(fontsize = ticks_size)
            plt.legend(['Estimation', 'Odometry', 'Camera', 'Accelerometer'], fontsize = legend_size)

            # ims.append([im1, im2, im3, im4, im5, im6, im7, im8])
            ims.append([im1, im2, im3, im4])
            
        ani = animation.ArtistAnimation(fig, ims, interval = 5)
        ani.save("test_single1.gif", writer='pillow')

    GIF_swarm = 0
    if(GIF_swarm):
        fig = plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
        plt.grid()
        ims = []
    
        temp_x = []
        temp_y = []
        for i in range(4):
            temp_x.append([])
            temp_y.append([])
        
        # print(temp_x)    
        for i in range(T):
        
            
            for j in range(4):
                a = pd.DataFrame(robot_history[str(j)]).T

                if(len(temp_x[j]) > 20):
                    temp_x[j].pop(0)
                    temp_y[j].pop(0)
                # print(a)
                temp_x[j].append(a["estimation_x"][i])
                temp_y[j].append(a["estimation_y"][i])
            # for j in range(2):
                
            #     im, = plt.plot(temp_x[j], temp_y[j], linestyle='--', marker='o', label='line with marker', markersize = 4)
            #     ims.append([im])
            im1, = plt.plot(temp_x[0], temp_y[0], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:blue')
            im2, = plt.plot(temp_x[1], temp_y[1], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:orange')
            im3, = plt.plot(temp_x[2], temp_y[2], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:green')
            im4, = plt.plot(temp_x[3], temp_y[3], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:red')
            # im5, = plt.plot(temp_x[4], temp_y[4], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:purple')
            # im6, = plt.plot(temp_x[5], temp_y[5], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:brown')
            # im7, = plt.plot(temp_x[6], temp_y[6], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:pink')
            # im8, = plt.plot(temp_x[7], temp_y[7], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:gray')

            a = pd.DataFrame(robot_history['0']).T
            im5, = plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=8, markeredgewidth = 4)
            # im6, = plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=8, markeredgewidth = 4)
    
            a = pd.DataFrame(robot_history['1']).T
            im7, = plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=8, markeredgewidth = 4)
            # im8, = plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=8, markeredgewidth = 4)
            
            a = pd.DataFrame(robot_history['2']).T
            im9, = plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=8, markeredgewidth = 4)
            # im10, = plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=8, markeredgewidth = 4)
            
            a = pd.DataFrame(robot_history['3']).T
            im11, = plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=8, markeredgewidth = 4)
            # im12, = plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=8, markeredgewidth = 4)
                        
            # print(type(im))
            # print(type(plt.gcf()))
            # im = plt.gcf()
            plt.xlabel("x position/m", fontsize = label_size)
            plt.ylabel("y position/m", fontsize = label_size)
            # plt.xlim([0, 0.6])
            # plt.ylim([0.5, 1.1])
            plt.title('Robots Tracking Result', fontsize = title_size)
            plt.yticks(fontsize = ticks_size)
            plt.xticks(fontsize = ticks_size)
            # plt.legend(['Estimation', 'Odometry', 'Camera', 'Accelerometer'], fontsize = legend_size)

            # ims.append([im1, im2, im3, im4, im5, im6, im7, im8])
            # ims.append([im1, im2, im3, im4, im5, im6, im7, im8, im9, im10, im11, im12])
            ims.append([im1, im2, im3, im4, im5, im7, im9, im11])
            
        ani = animation.ArtistAnimation(fig, ims, interval = 5)
        ani.save("test_swarm0.gif", writer='pillow')
        
        
        
    plt.show()        
                
    
    