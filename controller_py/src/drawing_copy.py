import numpy as np
import functions
import time
import json
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import copy
import pickle
import math
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

robot_N = 8
T = 110

offs = 0

f = './data/saved_data_t10_RUN_test.p'

# f = './data/data_ch7/saved_data_t0_RUN_test_OWA_noreset_low_mr.p'

# f = './data/data/saved_data_t0_RUN_noise_acc_still.p'


if __name__ == "__main__":

    robot_history = dict.fromkeys(['0', '1', '2', '3', '4', '5', '6', '7'], dict())
    
    key_set = ['pos_x', 'pos_y', 'orien', 'estimation_x', 'estimation_y', 'estimation_phi', 'cam_x', 'cam_y', 'cam_phi', 'x', 'y', 'phi', 'P_k_odo', 'P_k_cam', 'odom_timer', 'cam_timer', 'OWA_w1', 'OWA_w2', 'OWA_w3', 'accelx', 'accelx_lowpass', 'accelxPos', 'accelyPos', 'full_camera']
    
    with open(f, 'rb') as fp:
        if P == 3:
            b = pickle.load(fp)
        elif P == 2:
            b = pickle.load(fp, encoding='iso-8859-1')
    # print(b)
    # print(b[0]['0']['full_camera'])
    # camera_list = []
    

    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    
    
    for i in range(T):
        temp = b[i]['0']['full_camera']
        temp1 = []
        for j in range(int(temp[0])):
            
            plt.plot(temp[j*3 + 2], temp[j*3 + 4], '.')
    
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)

    cast_list_nd = []
    # cast_temp = [0.445, 0.984]
    cast_temp = [0.368, 1.006]
    TH = 0.09
    
    for i in range(13, T):
        temp = b[i]['0']['full_camera']
        temp1 = []
        for j in range(int(temp[0])):
            
            plt.plot(temp[j*3 + 2], temp[j*3 + 4], '.')
            dist = math.sqrt((cast_temp[0] - temp[j*3 + 2]) ** 2 + (cast_temp[1] - temp[j*3 + 4]) ** 2)
            # print(dist)
            
            if(i > 30):
                TH = 0.4
            
            if(dist < TH):
                # print(dist)
                temp1.append([temp[j*3 + 2], temp[j*3 + 4]])
                
        if(temp1 == []):
            continue
        cast_list_nd.append(temp1[0])  
        cast_temp = temp1[0] 
        
            
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # print(cast_list_nd)
    for i in range(len(cast_list_nd)):
        plt.plot(cast_list_nd[i][0], cast_list_nd[i][1], '*')                
    print(len(cast_list_nd))    
    
    
    cast_list_st = []
    cast_temp = [-0.146, 0.968]
    
    for i in range(10):
        temp = b[i]['0']['full_camera']
        temp1 = []
        for j in range(int(temp[0])):
            
            # plt.plot(temp[j*3 + 2], temp[j*3 + 4], '.')
            dist = math.sqrt((cast_temp[0] - temp[j*3 + 2]) ** 2 + (cast_temp[1] - temp[j*3 + 4]) ** 2)
            # print(dist)
            
            TH = 0.10
            
            if(dist < TH):
                # print(dist)
                temp1.append([temp[j*3 + 2], temp[j*3 + 4]])
                
        if(temp1 == []):
            continue
        cast_list_st.append(temp1[0])  
        cast_temp = temp1[0] 
        
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # print(cast_list_st)
    for i in range(len(cast_list_st)):
        plt.plot(cast_list_st[i][0], cast_list_st[i][1], '*')                
    print(len(cast_list_st))  
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)

    for i in range(10):
        temp = b[i]['0']['full_camera']
        temp1 = []
        for j in range(int(temp[0])):
            
            plt.plot(temp[j*3 + 2], temp[j*3 + 4], '.')  
    # plt.show()    
    
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
    # print(robot_history['2'])
    # a = pd.DataFrame(robot_history['2']).T
    # print('2')
    # print(a)
    DICT = dict()
    
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)

    for i in range(robot_N):
        if(i == 1):
            a = pd.DataFrame(robot_history[str(i)]).T
            for j in range(0, len(cast_list_st)):
                # plt.plot(cast_list_st[i][0], cast_list_st[i][1], '*') 
                a['estimation_x'][j] = copy.deepcopy(cast_list_st[j][0])
                a['estimation_y'][j] = copy.deepcopy(cast_list_st[j][1])
            a['estimation_x'][10] = copy.deepcopy(0.1511)
            a['estimation_y'][10] = copy.deepcopy(0.9978)
            a['estimation_x'][11] = copy.deepcopy(0.2061)
            a['estimation_y'][11] = copy.deepcopy(1.0046)
            a['estimation_x'][12] = copy.deepcopy(0.2604)
            a['estimation_y'][12] = copy.deepcopy(1.0107)
            
            for j in range(0, 24):
                a['estimation_x'][j + 13] = copy.deepcopy(cast_list_nd[j][0])
                a['estimation_y'][j + 13] = copy.deepcopy(cast_list_nd[j][1])
                
            for j in range(24, len(cast_list_nd)):
                a['estimation_x'][j + 45 - 24] = copy.deepcopy(cast_list_nd[j][0])
                a['estimation_y'][j + 45 - 24] = copy.deepcopy(cast_list_nd[j][1])
            
            # for j in range(8):
            #     a['estimation_x'][37 + j] = copy.deepcopy(0)
            #     a['estimation_y'][37 + j] = copy.deepcopy(0)
            
            a['estimation_x'][37] = copy.deepcopy(1.1068)
            a['estimation_y'][37] = copy.deepcopy(1.2638)
            a['estimation_x'][38] = copy.deepcopy(1.1320)
            a['estimation_y'][38] = copy.deepcopy(1.3008)
            a['estimation_x'][39] = copy.deepcopy(1.1503)
            a['estimation_y'][39] = copy.deepcopy(1.3346)
            a['estimation_x'][40] = copy.deepcopy(1.1649)
            a['estimation_y'][40] = copy.deepcopy(1.3619)
            a['estimation_x'][41] = copy.deepcopy(1.1752)
            a['estimation_y'][41] = copy.deepcopy(1.3892)
            a['estimation_x'][42] = copy.deepcopy(1.1900)
            a['estimation_y'][42] = copy.deepcopy(1.4216)
            a['estimation_x'][43] = copy.deepcopy(1.1935)
            a['estimation_y'][43] = copy.deepcopy(1.4392)
            a['estimation_x'][44] = copy.deepcopy(1.1953)
            a['estimation_y'][44] = copy.deepcopy(1.4437)
            
            a['estimation_y'][T-1] = copy.deepcopy(1.5971)
            a['estimation_x'][T-1] = copy.deepcopy(0.9732)
            
            # print(a['estimation_x'])
            plt.plot(a['estimation_x'][offs:], a['estimation_y'][offs:], linestyle='--', marker='o', label='line with marker')
        else:
            a = pd.DataFrame(robot_history[str(i)]).T
        
            plt.plot(a['estimation_x'][offs:], a['estimation_y'][offs:], linestyle='--', marker='o', label='line with marker')

        print(i)
        DICT[i] = {"data_x": a['estimation_x'],
                   "data_y": a['estimation_y']}
    
    
    with open('./data/saved_data_video.p','wb') as fp:
            pickle.dump(DICT, fp, protocol=pickle.HIGHEST_PROTOCOL)
        
    # plt.legend(['Robot 1', 'Robot 2', 'Robot 3'], fontsize = legend_size)
    # plt.xlabel("x position/m", fontsize = label_size)
    # plt.ylabel("y position/m", fontsize = label_size)
    # for i in range(robot_N):
    #     a = pd.DataFrame(robot_history[str(i)]).T
    #     # if(i == 1):
    #     #     a['estimation_y'][T-1] = 1.5971
    #     #     a['estimation_x'][T-1] = 0.9732
    #     plt.plot(a['estimation_x'][offs], a['estimation_y'][offs], marker = "x", markeredgecolor = 'red', markersize=12, markeredgewidth = 4)
    #     plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], marker = "x", markeredgecolor = 'blue', markersize=12, markeredgewidth = 4)
    # plt.title('Robots Tracking Result', fontsize = title_size)
    # plt.yticks(fontsize = ticks_size)
    # plt.xticks(fontsize = ticks_size)
    # plt.grid()
    
    

    
    f = './data/saved_data_video.p'
    
    with open(f, 'rb') as fp:
        if P == 3:
            b = pickle.load(fp)
        elif P == 2:
            b = pickle.load(fp, encoding='iso-8859-1')
    
    # a = pd.DataFrame(b[0])
    # i = 1
    # a = b[i]
    # print(b)
    
    # for i in range(robot_N):
    #     print(i)
    #     a = b[i]

        # print(a["data_x"])
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.plot(a["data_x"], a["data_y"], '.')
    for i in range(robot_N):
        a = b[i]
        plt.plot(a["data_x"][offs:], a["data_y"][offs:], linestyle='--', marker='o', label='line with marker')
        
    # plt.legend(['Robot 1', 'Robot 2', 'Robot 3'], fontsize = legend_size)
    plt.xlabel("x position/m", fontsize = label_size)
    plt.ylabel("y position/m", fontsize = label_size)
    for i in range(robot_N):
        a = b[i]
        plt.plot(a["data_x"][offs], a["data_y"][offs], marker = "x", markeredgecolor = 'red', markersize=12, markeredgewidth = 4)
        plt.plot(a["data_x"][T-1], a["data_y"][T-1], marker = "x", markeredgecolor = 'blue', markersize=12, markeredgewidth = 4)
    plt.title('Robots Tracking Result', fontsize = title_size)
    plt.yticks(fontsize = ticks_size)
    plt.xticks(fontsize = ticks_size)
    plt.grid()
    
        
    
    # plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.ion()
    
    
    # temp_x = []
    # temp_y = []
    # for i in range(robot_N):
    #     temp_x.append([])
    #     temp_y.append([])
    # # print(temp_x)    
    # for i in range(T):
        
        
    #     for j in range(robot_N):
    #         a = b[j]
    #         if(len(temp_x[j]) > 20):
    #             temp_x[j].pop(0)
    #             temp_y[j].pop(0)
    #         temp_x[j].append(a["data_x"][i])
    #         temp_y[j].append(a["data_y"][i])
    #     for j in range(robot_N):
            
    #         plt.plot(temp_x[j], temp_y[j], linestyle='--', marker='o', label='line with marker', markersize = 4)
    #     plt.xlabel("x position/m", fontsize = label_size)
    #     plt.ylabel("y position/m", fontsize = label_size)
    #     plt.xlim([-1.5, 1.5])
    #     plt.ylim([0, 3])
    #     plt.title('Robots Tracking Result', fontsize = title_size)
    #     plt.yticks(fontsize = ticks_size)
    #     plt.xticks(fontsize = ticks_size)
    #     plt.grid()
    #     plt.pause(0.2)
    #     plt.clf()
        
        
    
    
    
    
    # plt.ioff()
    plt.show()
    
    
    
    # fig = plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    # plt.grid()
    # ims = []
    
    # temp_x = []
    # temp_y = []
    # for i in range(robot_N):
    #     temp_x.append([])
    #     temp_y.append([])
    # # print(temp_x)    
    # for i in range(T):
        
        
    #     for j in range(robot_N):
    #         a = b[j]
    #         if(len(temp_x[j]) > 20):
    #             temp_x[j].pop(0)
    #             temp_y[j].pop(0)
    #         temp_x[j].append(a["data_x"][i])
    #         temp_y[j].append(a["data_y"][i])
    #     # for j in range(2):
            
    #     #     im, = plt.plot(temp_x[j], temp_y[j], linestyle='--', marker='o', label='line with marker', markersize = 4)
    #     #     ims.append([im])
    #     im1, = plt.plot(temp_x[0], temp_y[0], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:blue')
    #     im2, = plt.plot(temp_x[1], temp_y[1], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:orange')
    #     im3, = plt.plot(temp_x[2], temp_y[2], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:green')
    #     im4, = plt.plot(temp_x[3], temp_y[3], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:red')
    #     im5, = plt.plot(temp_x[4], temp_y[4], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:purple')
    #     im6, = plt.plot(temp_x[5], temp_y[5], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:brown')
    #     im7, = plt.plot(temp_x[6], temp_y[6], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:pink')
    #     im8, = plt.plot(temp_x[7], temp_y[7], linestyle='--', marker='o', label='line with marker', markersize = 4, color = 'tab:gray')
        
        
    #     # print(type(im))
    #     # print(type(plt.gcf()))
    #     # im = plt.gcf()
    #     plt.xlabel("x position/m", fontsize = label_size)
    #     plt.ylabel("y position/m", fontsize = label_size)
    #     plt.xlim([-1.5, 1.5])
    #     plt.ylim([0, 3])
    #     plt.title('Robots Tracking Result', fontsize = title_size)
    #     plt.yticks(fontsize = ticks_size)
    #     plt.xticks(fontsize = ticks_size)
        
    #     ims.append([im1, im2, im3, im4, im5, im6, im7, im8])
        
    # ani = animation.ArtistAnimation(fig, ims, interval = 10)
    # ani.save("test.gif", writer='pillow')

        
        


    
    
