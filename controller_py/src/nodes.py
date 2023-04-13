import utils

import rospy
import numpy as np
import json
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import copy
import pickle
import math
import typing

from configuration import *
from Kalman import *

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# for optitrack camera
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose


wheel_distance = 0.041
t_delay = 0.5
std_vlt_trans = 20
std_vlt_rot = 10

PI = math.pi

MODE = "cam"
MODE = "kalman"

sr_KALMAN = 1
mr_KALMAN = 0

with open('mapper.json') as json_file:
    mapper = json.load(json_file)

obstacle_avoidance = utils.ObstacleAvoidance()




class Camera: 
    def __init__(self, tag):
        
        self.cam_x = 0.0
        self.cam_y = 0.0
        self.cam_phi = 0.0
        self.tag = tag   
        self.timer = 0.0
        self.listener_camera = rospy.Subscriber('Bebop{}/ground_pose'.format(self.tag + 1), Pose2D, self.listen_optitrack_callback)
        self.listener_camera_timer = rospy.Subscriber('Bebop{}/pose'.format(self.tag + 1), Pose, self.listen_optitrack_timer_callback)
              
    # functions for subscribe callback    
    def listen_optitrack_callback(self, optiMsg):
        self.cam_x = optiMsg.x
        self.cam_y = optiMsg.y
        self.cam_phi = optiMsg.theta
        
    def listen_optitrack_timer_callback(self, optiMsg):
        self.timer = optiMsg.time
        
class Cameras:
    def __init__(self, N):
        self.number = N
        self.cameras = {tag: Camera(tag = tag) for tag in range(N)}
        self.measurement_list = np.zeros([N, 4])
        self.measurement_list_prev = np.zeros([N, 4])
        
    def update_camera(self):
        self.measurement_list = np.zeros([self.number, 4])
        i = 0
        for tag in self.cameras:
            self.measurement_list[i][0] = 2 - self.cameras[tag].cam_x
            self.measurement_list[i][1] = -self.cameras[tag].cam_y
            self.measurement_list[i][2] = self.cameras[tag].cam_phi
            self.measurement_list[i][3] = self.cameras[tag].timer
            i += 1
            
class Node:
    def __init__(self, release_time, range=range, tag=0):
        
        self.t = 0
        
        self.tag = tag
        self.release_time = release_time
        self.address = mapper[str(tag)]['address']
        self.start_pos = np.array(mapper[str(tag)]['pos'])
        self.start_orien = np.array(mapper[str(tag)]['orien'])

        # Initialize ros subscriber messages
        self.robot_meas_pose = np.array([np.NaN, np.NaN])
        self.robot_meas_orien = np.NaN
        self.robot_meas_time = np.NaN
        self.robot_input = np.NaN

        self.input_v = 0.0
        self.input_omega = 0.0

        self.x = self.start_pos[0]
        self.y = self.start_pos[1]
        self.phi = self.start_orien
        
        self.odom_x = self.x
        self.odom_y = self.y
        self.odom_phi = self.phi
        self.odom_timer = 0.0
        
        self.cam_x = self.x
        self.cam_y = self.y
        self.cam_phi = self.phi
        self.cam_timer = 0.0
        
        self.cam_prev = np.array([self.x, self.y, self.phi])
        self.MIN_dist_prev = 1
        
        self.theoretical_position = np.array([self.x, self.y, self.phi])
        self.estimation = np.array([self.x, self.y, self.phi])
        
        self.kalman_odo = Kalman()
        self.kalman_cam = Kalman()
        
        # buffer
        self.buffer_size = 5
        # self.camera_buffer = [[] for i in range(3)]

        self.odo_error_buffer = [0.1] * self.buffer_size
        self.camera_error_buffer = [0.1] * self.buffer_size
        
        self.goalX = np.array([0.0, 0.5])
        self.setup = {"vmax":0.5, "gtg_scaling":0.0001, "K_p":0.01, "ao_scaling":0.00005}

        # initialize theoretical positioning following buffer style
        self.pos = np.array([self.start_pos, self.start_pos, self.start_pos, self.start_pos, self.start_pos])
        self.orien = np.array([self.start_orien, self.start_orien, self.start_orien, self.start_orien, self.start_orien])

        # Initialize rostopic listeners/writers
        self.listener_robot_pose = rospy.Subscriber('elisa3_robot_{}/odom'.format(self.tag), Odometry,
                                                    self.listen_robot_pose_callback)
        
        # self.listener_camera = rospy.Subscriber('Bebop{}/ground_pose'.format(int(self.tag) + 1), Pose2D, self.listen_optitrack_callback)

        # Initialize shared update message attributes
        self.update_leds = False
        self.msg_leds = np.array([0,0,0])
        self.update_auto_motive = False
        self.msg_auto_motive = np.array([0,0,0,0])
        self.update_reset = False
        self.msg_reset = np.array([0, 0, 0, 0])
        self.trigger_auto_motive = 103
    
    # functions for LED
    def publish_greenLed(self, intensity = np.array([0])):
        self.update_leds = True
        self.msg_leds[0] = intensity

    def publish_redLed(self, intensity = np.array([0])):
        self.update_leds = True
        self.msg_leds[1] = intensity

    def publish_blueLed(self, intensity = np.array([0])):
        self.update_leds = True
        self.msg_leds[2] = intensity

    def led_off(self):
        self.publish_greenLed(intensity=np.array([0]))
        self.publish_blueLed(intensity=np.array([0]))
        self.publish_redLed(intensity=np.array([0]))
     
    def listen_robot_pose_callback(self, odomMsg):
        self.robot_meas_pose = np.round(np.array([float(odomMsg.pose.pose.position.x),
                                                  float(odomMsg.pose.pose.position.y)]),3)
        self.robot_meas_orien = np.round(float(odomMsg.pose.pose.position.z),3)
        self.odom_timer = odomMsg.header.stamp.secs
        self.odom_x = float(odomMsg.pose.pose.position.x)
        self.odom_y = float(odomMsg.pose.pose.position.y)
        self.odom_phi = float(odomMsg.pose.pose.position.z)
    
    # # functions for subscribe callback    
    # def listen_optitrack_callback(self, optiMsg):
    #     self.cam_x = optiMsg.x
    #     self.cam_y = -optiMsg.y
    #     self.cam_phi = optiMsg.theta
    #     # print(self.cam_x)

    def print_position_measures(self):
        msg = """ 
                ID: {}
                Estimation: 
                position: {} - orientation: {}
                Odom Measure: 
                position: {} - orientation: {}
                Cam Measure: 
                position: {} - orientation: {}
                  """.format(self.tag,
                            #  self.pos[-1], self.orien[-1],
                            #  self.robot_meas_pose, self.robot_meas_orien)
                            [self.estimation[0], self.estimation[1]], self.estimation[2],
                            [self.odom_x, self.odom_y], self.odom_phi,
                            [self.cam_x, self.cam_y], self.cam_phi)
        # if(self.tag == '0'):            
        #     print("msg: ", msg)
        print("msg: ", msg)

    def compute_move(self, pol: np.array):
        """
        :param pol: intended change in location nin polar coordinates np.array([rho, phi]) g
        :return: ros instruction [left or right, degrees to turn, forward or backward, longitudinal move]
        """
        # self.print_position_measures()
        orien_cor = self.orien[-1]
        if orien_cor < 0:
            orien_cor += 2 * np.pi
        orien_cor %= (2 * np.pi)
        if orien_cor > np.pi:
            orien_cor -= 2*np.pi

        phi_cor = pol[1]
        if phi_cor < 0:
            phi_cor += 2 * np.pi
        phi_cor %= (2 * np.pi)
        if phi_cor > np.pi:
            phi_cor -= 2*np.pi

        delta_phi = phi_cor - orien_cor

        if delta_phi >= 0:
            self.msg_auto_motive = np.array([1, delta_phi, 0, pol[0]])
        else:
            self.msg_auto_motive = np.array([0, -delta_phi, 0, pol[0]])

        self.update_auto_motive = True

        if self.trigger_auto_motive == 103:
            self.trigger_auto_motive = 104
        else:
            self.trigger_auto_motive = 103

        # UPDATE POSITION
        prop_move_cart = utils.pol2cart(np.array([pol[0], phi_cor]))
        calc_point, obs_avoid_mode = obstacle_avoidance.obstacle_avoidance(self.pos[-1], prop_move_cart)

        self.pos = utils.renew_vec(self.pos)
        self.orien = utils.renew_vec(self.orien)

        self.pos[-1] = calc_point
        self.orien[-1] += delta_phi

    def reset(self, type = 'odom'):
        if type == 'odom':
            if np.isnan(sum(self.robot_meas_pose)) or np.isnan(self.robot_meas_orien):
                print('robot: {} odom measures give nan, theor kept')
            else:
                self.pos[-1] = self.robot_meas_pose
                self.orien[-1] = self.robot_meas_orien
                
            self.update_reset = False
        elif type == 'theor':
            self.update_reset = True
            # self.msg_reset = np.array([0, self.pos[-1][0], self.pos[-1][1], self.orien[-1]])
            self.msg_reset = np.array([0, self.estimation[0], self.estimation[1], self.estimation[2]])
            
    
    def states_transform(self, X, v, omega):
        X[0] = X[0] + v * math.cos(X[2]) / 1000 
        X[1] = X[1] + v * math.sin(X[2]) / 1000 
        X[2] = X[2] + omega
        return X

    def go_to_goal(self):
        e = self.goalX - [self.estimation[0], self.estimation[1]]     # error in position

        K_P = [-50, -50]
        v = np.linalg.norm(K_P * e)   # Velocity decreases as bot gets closer to goal
        
        phi_d = math.atan2(e[1], e[0])  # Desired heading
        
        omega = self.setup["K_p"]*math.atan2(math.sin(phi_d - self.estimation[2]), math.cos(phi_d - self.estimation[2]))     # Only P part of a PID controller to give omega as per desired heading
        
        return [v, omega]

    def ternimate(self):
        MAX_length = 0.001
        distance = math.sqrt((self.goalX[0] - self.estimation[0])**2 + (self.goalX[1] - self.estimation[1])**2)
        # print(distance)
        if(distance < MAX_length):
            self.moving = 0
            return 0
        else:
            self.moving = 1
            return 1    
    
    def reset_goal(self):
        [self.input_v, self.input_omega] = [0.0, 0.0]
    
    def determine_camera(self, cameras):
        # update the camera
        MIN_dist = 1e5
        cameras.update_camera()
        
        # print("estimation: ", self.estimation)

        i = 0; idx = 0
        for item in cameras.measurement_list:
            dist = math.sqrt((self.estimation[0] - item[0])**2 + (self.estimation[1] - item[1])**2)
            print("est: ", [self.estimation[0], self.estimation[1]])
            print('\n')
            print("cam: ", [item[0], item[1]])
            print('\n')
            print("dist", dist)
            print("MIN_dist", MIN_dist)
            
            if(dist < MIN_dist):
                MIN_dist = dist
                idx = i
            i += 1

        
        dist_odo_cam = math.sqrt((self.odom_x - cameras.measurement_list[idx][0])**2 + (self.odom_y - cameras.measurement_list[idx][1])**2)
        # print("odom: ", [self.odom_x, self.odom_y])
        # print("cam: ", [cameras.measurement_list[idx][0], cameras.measurement_list[idx][1]])
        
        # print("dist_odo_cam", dist_odo_cam)
        
        # if(MIN_dist > 1 or dist_odo_cam > 0.3):
        
        
        # self.cam_x = cameras.measurement_list[idx][0]
        # self.cam_y = cameras.measurement_list[idx][1]
        # self.cam_phi = cameras.measurement_list[idx][2]
        self.cam_timer = cameras.measurement_list[idx][3]
        
        
        if(self.t == 1):
            self.cam_x = cameras.measurement_list[idx][0]
            self.cam_y = cameras.measurement_list[idx][1]
            self.cam_phi = cameras.measurement_list[idx][2]
            self.MIN_dist_prev = MIN_dist
            return [self.cam_x, self.cam_y, self.odom_phi]
        
        print('\n')
        print("camera: ", [self.cam_x, self.cam_y, self.odom_phi])
        print('\n')
        
        error = abs(MIN_dist - self.MIN_dist_prev) / self.MIN_dist_prev
        print("error", error)
        if(error > 0.9 or MIN_dist > 1):
            return [self.odom_x, self.odom_y, self.odom_phi]
        else:
            self.cam_x = cameras.measurement_list[idx][0]
            self.cam_y = cameras.measurement_list[idx][1]
            self.cam_phi = cameras.measurement_list[idx][2]
            return [self.cam_x, self.cam_y, self.odom_phi]
    
    def measurement_update(self, cameras):
        
        # take the measurement from the odom
        odom_measurement = [self.odom_x, self.odom_y, self.odom_phi]        
        
        # take the measurement from the odom
        cam_measurement = self.determine_camera(cameras)
        [self.cam_x, self.cam_y, self.cam_phi] = cam_measurement
        # # for test
        # [self.cam_x, self.cam_y, self.odom_phi] = [self.odom_x, self.odom_y, self.odom_phi]   
        # cam_measurement = [self.odom_x, self.odom_y, self.odom_phi]      
        
        self.print_position_measures()
        
        return [odom_measurement, cam_measurement]
            
    def measurement_fusion(self, odom_measurement, cam_measurement):
        if (MODE == "cam"):
            self.estimation = cam_measurement
            return 
        
        self.kalman_odo.R_k = np.array([[1.0,   0,    0],
                                     [  0, 1.0,    0],
                                     [  0,    0, 1.0]]) 
        self.kalman_odo.Q_k = np.array([[0.01,   0,    0],
                                     [  0, 0.01,    0],
                                     [  0,    0, 0.005]]) 
        optimal_state_estimate_k, covariance_estimate_k = self.kalman_odo.sr_EKF(odom_measurement, self.estimation, 1)
          
        self.measurement_Kalman = optimal_state_estimate_k
        self.kalman_odo.P_k_1 = covariance_estimate_k
        self.estimation = self.measurement_Kalman
        
        if (self.t % 5 == 0):
            if(sr_KALMAN and ~mr_KALMAN):
                optimal_state_estimate_k, covariance_estimate_k = self.kalman_cam.sr_EKF(cam_measurement, self.estimation, 1)
            elif(mr_KALMAN and ~sr_KALMAN):
                optimal_state_estimate_k, covariance_estimate_k = self.kalman_cam.mr_EKF(cam_measurement, self.estimation, 1)
            self.measurement_Kalman = optimal_state_estimate_k
            self.kalman_cam.P_k_1 = covariance_estimate_k
            self.estimation = self.measurement_Kalman
            
    def measurement_fusion_OWA(self, odom_measurement, cam_measurement):
        if (MODE == "cam"):
            self.estimation = cam_measurement
            return 
            
        self.kalman_odo.R_k = np.array([[1.0,   0,    0],
                                     [  0, 1.0,    0],
                                     [  0,    0, 1.0]]) 
        self.kalman_odo.Q_k = np.array([[0.0001,   0,    0],
                                     [  0, 0.0001,    0],
                                     [  0,    0, 0.0001]]) 
        optimal_state_estimate_k, covariance_estimate_k = self.kalman_odo.sr_EKF(odom_measurement, self.estimation, 1)
          
        self.measurement_Kalman = optimal_state_estimate_k
        self.kalman_odo.P_k_1 = covariance_estimate_k
        self.estimation = self.measurement_Kalman
        
        odo_estimation = self.measurement_Kalman
        
        
        if (self.t % 1 == 0):
            if(sr_KALMAN and ~mr_KALMAN):
                optimal_state_estimate_k, covariance_estimate_k = self.kalman_cam.sr_EKF(cam_measurement, self.estimation, 1)
            elif(mr_KALMAN and ~sr_KALMAN):
                optimal_state_estimate_k, covariance_estimate_k = self.kalman_cam.mr_EKF(cam_measurement, self.estimation, 1)
            self.measurement_Kalman = optimal_state_estimate_k
            self.kalman_cam.P_k_1 = covariance_estimate_k
            self.estimation = self.measurement_Kalman     
            
            
            cam_estimation = self.measurement_Kalman
            
            
            err_camera = ((self.estimation[0] - self.cam_x)**2 + (self.estimation[1] - self.cam_y)**2)
            err_odo = ((self.estimation[0] - self.odom_x)**2 + (self.estimation[1] - self.odom_y)**2)
            
            
            self.camera_error_buffer.append(err_camera)
            self.camera_error_buffer.pop(0)

            
            self.odo_error_buffer.append(err_odo)
            self.odo_error_buffer.pop(0)
            
            
            
            sum_camera = sum(self.camera_error_buffer)
            sum_odo = sum(self.odo_error_buffer)
            
            
                
            w1 = sum_camera / (sum_camera + sum_odo + 1) 
            w2 = (sum_odo + 1) / (sum_camera + sum_odo + 1)
            
            w2 = 0.99
            w1 = 0.01
            
            # if(sum_camera > 30 * self.buffer_size):
            #     w1 = 0.9
            #     w2 = 0.1
            
            # w1 = 0.2
            # w2 = 0.8
            
            self.OWA_w1 = w1
            self.OWA_w2 = w2
            
            self.estimation = w2 * cam_estimation + w1 * odo_estimation  
        
    
    def loop_fuc(self, cameras, move_type):
        
        # update the timer
        self.t += 1
        
        # 1. take measurement from odom and cam odom_measurement and cam_measurement via sub
        [odom_measurement, cam_measurement] = self.measurement_update(cameras)
        
        # 2. estimated the position
        self.measurement_fusion_OWA(odom_measurement, cam_measurement)
        # self.estimation = [self.cam_x, self.cam_y, self.cam_phi]
        
        # 3. decide where to go based on self.estimation
        # could add a logic to let robor decide where to go 
        if(move_type == 'move'):     
            [step, omega] = self.go_to_goal()
            [step, omega] = [1.5, 0]
        else:
            step = 0
            omega = 0
        
        if(self.ternimate() == 0):
            step = 0
            omega = 0
            
        self.input_v = step
        self.input_omega = omega
        
        # execution
        
        self.compute_move(pol = np.array([step, omega]))
        
        # self.theoretical_position = self.states_transform(self.theoretical_position, step, omega)
        # self.estimation = self.states_transform(self.estimation, step, omega)
        
                               
class Nodes:
    def __init__(self, active_robots = ['0000']):
        for tag in active_robots:
            if tag not in mapper:
                print('mapper robot: ' + str(tag) +' is missing ')
                break
            
        rospy.init_node('python_node') 
                   
        self.active_robots = active_robots

        self.N_total = len(active_robots)

        
        self.publisher_auto_motive = rospy.Publisher("elisa3_all_robots/auto_motive", Float64MultiArray,
                                                     queue_size=10)
        self.publisher_leds = rospy.Publisher("elisa3_all_robots/leds", Float64MultiArray,
                                                     queue_size=10)
        self.publisher_reset = rospy.Publisher("elisa3_all_robots/reset", Float64MultiArray,
                                              queue_size=10)
        
        self.publisher_input = rospy.Publisher("mobile_base/input", Twist,
                                              queue_size=10)
        
        self.publisher_inputs = rospy.Publisher("mobile_base/inputs", Twist,
                                              queue_size=10)
        
        self.msg_input = Twist()
        self.msg_inputs = Float64MultiArray()
        
        self.msg_auto_motive = Float64MultiArray()
        self.msg_leds = Float64MultiArray()
        self.msg_reset = Float64MultiArray()

        self.nodes = {tag: Node(release_time=0, tag=tag) for tag in active_robots}
        
        self.cameras = Cameras(self.N_total)
        

        # store
        self.saved_data = dict()
        
        # plot
        self.buffer = 10
        self.ax = np.zeros((1, self.buffer))[0]
        self.ay = np.zeros((1, self.buffer))[0]
    
    def loop_fuc(self, move_type = 'move'):
        # self.cameras.update_camera()
        for tag in self.nodes:
            self.nodes[tag].loop_fuc(self.cameras, move_type)
            
        # SETUP MESSAGE
        self.msg_auto_motive.data = np.array([len(self.nodes)])
        for tag in self.nodes:
            self.msg_auto_motive.data = np.concatenate((self.msg_auto_motive.data,
                    np.array([int(self.nodes[tag].address)]), self.nodes[tag].msg_auto_motive), axis=0)

        # SEND MOVE MESSAGE
        # rospy.sleep(1)
        self.publisher_auto_motive.publish(self.msg_auto_motive)
        rospy.sleep(0.1)            
    
    def test_cam(self):
        self.cameras.update_camera()
        for i in range(self.N_total):
            print("camera: ", self.cameras.measurement_list[i])
            
        for tag in self.nodes:
            cam_measurement = self.nodes[tag].determine_camera(self.cameras)
            print("tag: ", tag)
            print(cam_measurement)
    
    def print_fuc(self):
        for tag in self.nodes:
            self.nodes[tag].print_position_measures()

    def move(self, move_type = 'move', step_size: float = 0., theta: float =0.0):
        for tag in self.nodes:
                             
            if(move_type == 'move'):     
                [step, omega] = self.nodes[tag].go_to_goal()
            else:
                step = step_size
                omega = theta
                        
            self.nodes[tag].compute_move(pol = np.array([step, omega])) # \TODO change to move
        self.print_position_measures()    
        # SETUP MESSAGE
        self.msg_auto_motive.data = np.array([len(self.nodes)])
        for tag in self.nodes:
            self.msg_auto_motive.data = np.concatenate((self.msg_auto_motive.data,
                    np.array([int(self.nodes[tag].address)]), self.nodes[tag].msg_auto_motive), axis=0)

        # SEND MOVE MESSAGE
        rospy.sleep(0.1)
        self.publisher_auto_motive.publish(self.msg_auto_motive)
        # rospy.sleep(10)

    def update_leds(self, subset_tags: typing.Optional[list] = None):
        self.msg_leds.data = np.array([0])
        count = 0

        if subset_tags:
            ToIterateOver = subset_tags
        else:
            ToIterateOver = self.nodes.keys()

        for tag in ToIterateOver:
            self.msg_leds.data = np.concatenate((self.msg_leds.data, np.array([int(self.nodes[tag].address)]),
                             self.nodes[tag].msg_leds), axis=0)
            count += 1
        self.msg_leds.data[0] = count

        self.publisher_leds.publish(self.msg_leds)

    def reset(self, type = 'odom', subset_tags: typing.Optional[list] = None):
        self.msg_reset.data = np.array([0])

        if subset_tags:
            ToIterateOver = subset_tags
        else:
            ToIterateOver = self.nodes.keys()

        count = 0
        for tag in ToIterateOver:
            self.nodes[tag].reset(type=type)

            # ONLY REPLENISH MESSAGE IF THEOR UPDATE TYPE
            if self.nodes[tag].update_reset:
                self.msg_reset.data = np.concatenate((self.msg_reset.data, np.array([int(self.nodes[tag].address)]),
                                                      self.nodes[tag].msg_reset), axis=0)
                count += 1

        if count != 0:
            print("reset")
            self.msg_reset.data[0] = count
            self.publisher_reset.publish(self.msg_reset)

    def turn_off_leds(self):
        for tag in self.nodes:
            self.nodes[tag].led_off()
        self.update_leds()

    def set_leds(self, green=0, blue=0, red=0):
        for tag in self.nodes:
            self.nodes[tag].publish_greenLed(intensity=np.array([green]))
            self.nodes[tag].publish_blueLed(intensity=np.array([blue]))
            self.nodes[tag].publish_redLed(intensity=np.array([red]))
        self.update_leds()
    
    def print_position_measures(self):
        for tag in self.nodes:
            self.nodes[tag].print_position_measures()

    def store_data(self, t):
        self.saved_data[t] = dict()
        for tag in self.nodes:
            # self.saved_data[t][tag] = {'pos_x': copy.deepcopy(self.nodes[tag].pos[-1][0]),
            #                            'pos_y': copy.deepcopy(self.nodes[tag].pos[-1][1]),
            #                            'orien': copy.deepcopy(self.nodes[tag].orien[-1])
            #                            }
            self.saved_data[t][tag] = {'pos_x': copy.deepcopy(self.nodes[tag].odom_x),
                                       'pos_y': copy.deepcopy(self.nodes[tag].odom_y),
                                       'orien': copy.deepcopy(self.nodes[tag].odom_phi),
                                       'estimation_x': copy.deepcopy(self.nodes[tag].estimation[0]),
                                       'estimation_y': copy.deepcopy(self.nodes[tag].estimation[1]),
                                       'estimation_phi': copy.deepcopy(self.nodes[tag].estimation[2]),
                                       'cam_x': copy.deepcopy(self.nodes[tag].cam_x),
                                       'cam_y': copy.deepcopy(self.nodes[tag].cam_y),
                                       'cam_phi': copy.deepcopy(self.nodes[tag].cam_phi),
                                       'x': copy.deepcopy(self.nodes[tag].theoretical_position[0]),
                                       'y': copy.deepcopy(self.nodes[tag].theoretical_position[1]),
                                       'phi': copy.deepcopy(self.nodes[tag].theoretical_position[2]),
                                       'P_k_odo': copy.deepcopy(self.nodes[tag].kalman_odo.P_k_1),
                                       'P_k_cam': copy.deepcopy(self.nodes[tag].kalman_cam.P_k_1),
                                       'odom_timer': copy.deepcopy(self.nodes[tag].odom_timer),
                                       'cam_timer': copy.deepcopy(self.nodes[tag].cam_timer)
                                       }
        self.ax[ :-1] = self.ax[ 1:]
        self.ay[ :-1] = self.ay[ 1:]
        
        self.ax[-1] = self.nodes['0'].estimation[0]
        self.ay[-1] = self.nodes['0'].estimation[1]

    def save_data(self, t):
        with open('./data/saved_data_t{}_RUN{}.p'.format(t, 1),'wb') as fp:
            pickle.dump(self.saved_data, fp, protocol=pickle.HIGHEST_PROTOCOL)

    def plot_data(self, t):
        plt.ion()
        plt.clf()
        plt.plot(self.ax, self.ay, '.')
        
        plt.pause(0.1)
        plt.ioff()




