import numpy as np
import matplotlib.pyplot as plt
import os.path
from os import path

filename = "data_run0.txt"
data = np.loadtxt(filename, comments="#", delimiter=",")

N = 3

id = data[:, 0]

t = np.zeros([N,100000])
theta = np.zeros([N,100000])
theta_filtered = np.zeros([N,100000])
robtheta = np.zeros([N,100000])
robtheta_filtered = np.zeros([N,100000])
xpos_filtered = np.zeros([N,100000])
xpos = np.zeros([N,100000])
ypos = np.zeros([N,100000])
ypos_filtered = np.zeros([N,100000])
turn = np.zeros([N,100000])
estimate = np.zeros([N,100000])
length = np.zeros(N, dtype=int)
nb_events = np.zeros([N,100000])
nb_resets = np.zeros([N,100000])

for i in range(N):
    length[i] = int(np.where(id==i)[0].size)
    t[i, :length[i]] = data[np.where(id == i)[0], 1]
    robtheta[i, :length[i]] = data[np.where(id == i)[0], 2]
    robtheta_filtered[i, :length[i]] = data[np.where(id == i), 3]
    xpos[i, :length[i]] = data[np.where(id == i), 5]
    xpos_filtered[i, :length[i]] = data[np.where(id == i), 4]
    ypos[i, :length[i]] = data[np.where(id == i), 7]
    ypos_filtered[i, :length[i]] = data[np.where(id == i), 6]
    turn[i, :length[i]] = data[np.where(id == i), 8]
    theta[i, :length[i]] = data[np.where(id == i)[0], 10]
    theta_filtered[i, :length[i]] = data[np.where(id == i), 9]
    estimate[i, :length[i]] = data[np.where(id == i), 15]
    nb_events[i, :length[i]] = data[np.where(id == i), 12]
    nb_resets[i, :length[i]] = data[np.where(id == i), 13]
    # theta[i, :length[i]] = data[np.where(id == i)[0], 16]
    # theta_filtered[i, :length[i]] = data[np.where(id == i), 17]


# t = data[:,1]
# theta = data[:,2]
# theta_filter = data[:,3]
# xpos_filtered = data[:,4]
# xpos = data[:,5]
# ypos_filtered = data[:,6]
# ypos = data[:,7]

# butterworth filter
# theta_test = np.zeros(length[0])
# for m in range(1,length[0]):
#     theta_test[m] = (10*theta_test[m-1] + theta[0,m-1] + theta[0,m])/12




# for i in range(N):
#     plt.figure()
#     plt.plot(t[i,:length[i]]-t[i,0], robtheta[i, :length[i]], '.')
#     plt.plot(t[i,:length[i]]-t[i,0], robtheta_filtered[i, :length[i]], '.')
#     plt.plot(t[i,:length[i]]-t[i,0], theta_filtered[i, :length[i]], '.')
#     #plt.plot(t[i,:length[i]]-t[i,0], theta[i, :length[i]], '.')
#     plt.plot(t[i,:length[i]]-t[i,0], estimate[i,:length[i]]*200, '.')
#     plt.plot(t[i, :length[i]] - t[i, 0], robtheta[i, :length[i]], 'k', alpha=0.2)
#     plt.plot(t[i, :length[i]] - t[i, 0], robtheta_filtered[i, :length[i]], 'k', alpha=0.2)
#     plt.plot(t[i, :length[i]] - t[i, 0], theta_filtered[i, :length[i]], 'k', alpha=0.2)
#     #plt.plot(t[i, :length[i]] - t[i, 0], theta[i, :length[i]], 'k', alpha=0.2)
#     plt.plot(t[i,:length[i]]-t[i,0], estimate[i,:length[i]]*200, 'k', alpha=0.2)
#
#     # plt.legend(["theta robot", "filtered theta robot", "filtered theta pc", "estimate theta pc"])
#     plt.legend(["theta robot", "filtered theta robot", "filtered theta pc", "estimate"])
#     plt.title("theta from robot %d" % i)

# plt.figure()
# for i in range(N):
#     plt.plot(t[i,:length[i]]-t[i,0], theta_filtered[i, :length[i]], '.')
#     plt.plot(t[i, :length[i]] - t[i, 0], theta_filtered[i, :length[i]], 'k', alpha=0.2)
# #plt.legend(["0","1","2"])
# plt.title("theta (filtered) from robot")



# plt.figure()
# plt.plot((t-t[0])/1000, theta)
# plt.plot((t-t[0])/1000, theta_filter, 'c')
#plt.axhline(10, color='r')

# plt.figure()
# for i in range(N):
#     plt.plot(xpos_filtered[i,:length[i]], ypos_filtered[i,:length[i]], '.')
#     plt.plot(xpos_filtered[i, :length[i]], ypos_filtered[i, :length[i]], 'k', alpha=0.2)
# #plt.legend(["0","1","2"])
# plt.ylim([0,3000])
# plt.xlim([0,3000])
# plt.title("xpos and ypos (filtered)")



# plt.figure()
# for i in range(N):
#     plt.plot(xpos[i,:length[i]], ypos[i,:length[i]], '.')
#     plt.plot(xpos[i, :length[i]], ypos[i, :length[i]], 'k', alpha=0.2)
#     plt.ylim([0,2000])
#     plt.xlim([0,2000])
# #plt.legend(["0","1","2"])
# plt.title("xpos and ypos")


for i in range(N):
    plt.figure()
    plt.plot(t[i,:length[i]]-t[i,0], ypos[i,:length[i]], '.')
    plt.plot(t[i,:length[i]]-t[i,0], ypos_filtered[i,:length[i]], '.')
    plt.plot(t[i,:length[i]]-t[i,0], ypos_filtered[i, :length[i]], 'k', alpha=0.2)
    plt.plot(t[i,:length[i]]-t[i,0], ypos[i, :length[i]], 'k', alpha=0.2)
    plt.plot(t[i,:length[i]]-t[i,0], estimate[i,:length[i]]*2000, '.')
    plt.plot(t[i,:length[i]]-t[i,0], estimate[i, :length[i]]*2000, 'k', alpha=0.2)
    plt.ylim([0, 2000])
    # plt.xlim([0, 2000])
    #plt.legend(["0","1","2"])
    plt.title("Robot %d" %i + " ypos (filtered)")


    plt.figure()
    plt.plot(t[i,:length[i]]-t[i,0], xpos[i,:length[i]], '.')
    plt.plot(t[i,:length[i]]-t[i,0], xpos_filtered[i,:length[i]], '.')
    plt.plot(t[i,:length[i]]-t[i,0], xpos_filtered[i, :length[i]], 'k', alpha=0.2)
    plt.plot(t[i,:length[i]]-t[i,0], xpos[i, :length[i]], 'k', alpha=0.2)
    plt.plot(t[i,:length[i]]-t[i,0], estimate[i,:length[i]]*2000, '.')
    plt.plot(t[i,:length[i]]-t[i,0], estimate[i, :length[i]]*2000, 'k', alpha=0.2)
    plt.ylim([0, 2000])
    # plt.xlim([0, 2000])
    #plt.legend(["0","1","2"])
    plt.title("Robot %d" %i + " xpos (filtered)")


    plt.figure()
    plt.plot(t[i,:length[i]]-t[i,0], nb_events[i,:length[i]], '.')
    plt.plot(t[i,:length[i]]-t[i,0], nb_events[i, :length[i]], 'k', alpha=0.2)
    plt.plot(t[i, :length[i]] - t[i, 0], nb_resets[i, :length[i]], '.')
    plt.plot(t[i, :length[i]] - t[i, 0], nb_resets[i, :length[i]], 'k', alpha=0.2)
    #plt.legend(["0","1","2"])
    plt.ylim([0, 1000])
    plt.title("Robot %d" %i + " events")

plt.show()

