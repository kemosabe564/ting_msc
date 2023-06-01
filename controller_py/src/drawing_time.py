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

f = 'controller_py/src/my_recording4.txt'

# f = './data/data_ch7/saved_data_t0_RUN_test_OWA_noreset_low_mr.p'

# f = './data/data/saved_data_t0_RUN_noise_acc_still.p'
# import os

# cwd = os.getcwd()  # Get the current working directory (cwd)
# files = os.listdir(cwd)  # Get all the files in that directory
# print("Files in %r: %s" % (cwd, files))

N = 30
robot_n = 4
offs = 3
if __name__ == "__main__":
    a = []
    with open(f, 'r') as file:
        a = file.readlines()
    # print(a[offs + 3*robot_n][2:])
    
    
    time_in = []
    time_between = []
    
    time_prev = float(a[3][2:])
    # print(float(a[offs + 3*robot_n-1][2:]))
    # print(float(a[offs + 0][2:]))
    # time1 = (float(a[offs + 3*robot_n-1][2:]) - float(a[offs + 0][2:]))
    # print(time1)
    
    # print(a[14][2:])
    
    # for i in range(22):
    #     print("value: ", a[i][2:])
    for i in range(N):
        # print(i)
        time1 = (float(a[offs + 3*robot_n - 1][2:]) - float(a[offs + 0][2:]))
        # print(time1)
        time_in.append(time1)
        
        time2 = float(a[offs + 3*robot_n - 1][2:]) - time_prev
        time_between.append(time2)
        time_prev = float(a[offs + 0][2:])
        
        offs += (5 + 3*robot_n)
    
    
    # print(time_in)    
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    plt.plot(time_in, 'x')
    
    print(sum(time_in)/30)
    plt.figure(figsize = (figsize_x, figsize_y), dpi = dpi_number)
    plt.plot(time_between, 'x')
    print(sum(time_between)/30)
    plt.show()