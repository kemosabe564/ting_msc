import utils

import rospy
import numpy as np
import json
import matplotlib
matplotlib.use('Agg')
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


wheel_distance = 0.041
t_delay = 0.5
std_vlt_trans = 20
std_vlt_rot = 10

PI = math.pi

with open('mapper.json') as json_file:
    mapper = json.load(json_file)

obstacle_avoidance = utils.ObstacleAvoidance()

class Node:
    def __init__(self, release_time, range=range, tag=0):
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
        
        self.cam_x = self.x
        self.cam_y = self.y
        self.cam_phi = self.phi
        
        self.estimation = np.array([self.x, self.y, self.phi])
               
        self.goalX = np.array([0.5, 0.0])
        self.setup = {"vmax":0.5, "gtg_scaling":0.0001, "K_p":0.01, "ao_scaling":0.00005}

        # initialize theoretical positioning following buffer style
        self.pos = np.array([self.start_pos, self.start_pos, self.start_pos, self.start_pos, self.start_pos])
        self.orien = np.array([self.start_orien, self.start_orien, self.start_orien, self.start_orien, self.start_orien])

        # Initialize rostopic listeners/writers
        self.listener_robot_pose = rospy.Subscriber('elisa3_robot_{}/odom'.format(self.tag), Odometry,
                                                    self.listen_robot_pose_callback)
        
        self.listener_camera = rospy.Subscriber('Bebop{}/ground_pose'.format(1), Pose2D, self.listen_optitrack_callback)

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
        self.robot_meas_time = odomMsg.header.stamp.secs
        self.odom_x = float(odomMsg.pose.pose.position.x)
        self.odom_y = float(odomMsg.pose.pose.position.y)
        self.odom_phi = float(odomMsg.pose.pose.position.z)
    
    # functions for subscribe callback    
    def listen_optitrack_callback(self, optiMsg):
        self.cam_x = optiMsg.x
        self.cam_y = optiMsg.y
        self.cam_phi = optiMsg.theta

    def print_position_measures(self):
        msg = """ 
                ID: {}
                Theoretical: 
                position: {} - orientation: {}
                Odom Measure: 
                position: {} - orientation: {}
                  """.format(self.tag,
                             self.pos[-1], self.orien[-1],
                             self.robot_meas_pose, self.robot_meas_orien)
        print("msg: ", msg)

    def compute_move(self, pol: np.array):
        """
        :param pol: intended change in location nin polar coordinates np.array([rho, phi]) g
        :return: ros instruction [left or right, degrees to turn, forward or backward, longitudinal move]
        """
        self.print_position_measures()

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
            self.msg_reset = np.array([0, self.pos[-1][0], self.pos[-1][1], self.orien[-1]])
    
    
    
    
    def states_transform(self, X, v, omega):
        X[0] = X[0] + v * math.cos(X[2])# *2/3/1000/4
        X[1] = X[1] + v * math.sin(X[2])# *2/3/1000
        X[2] = X[2] + omega
        return X

    def go_to_goal(self):
        e = self.goalX - [self.estimation[0], self.estimation[1]]     # error in position

        # K_P = self.data["vmax"] * (1 - np.exp(- self.data["gtg_scaling"] * np.linalg.norm(e)**2)) / np.linalg.norm(e)     # Scaling for velocity
        K_P = [-10, -10]
        v = np.linalg.norm(K_P * e)   # Velocity decreases as bot gets closer to goal
        
        phi_d = math.atan2(e[1], e[0])  # Desired heading
        
        omega = self.setup["K_p"]*math.atan2(math.sin(phi_d - self.estimation[2]), math.cos(phi_d - self.estimation[2]))     # Only P part of a PID controller to give omega as per desired heading
        
        # omega_MAX = 1 * PI
        # if(omega > omega_MAX):
        #     omega = omega_MAX
        # elif(omega < -omega_MAX):
        #     omega = -omega_MAX
        
        return [v, omega]

    def ternimate(self):
        MAX_length = 0.001
        distance = math.sqrt((self.goalX[0] - self.estimation[0])**2 + (self.goalX[1] - self.estimation[1])**2)
        if(distance < MAX_length):
            self.moving = 0
            return 0
        else:
            self.moving = 1
            return 1    
    
    def reset_goal(self):
        [self.input_v, self.input_omega] = [0.0, 0.0]
    
    def loop_fuc(self):
        self.ternimate()
        if(self.moving == 1):
        
            # do the fusing to get the estimation
            
            self.estimation = [self.odom_x, self.odom_y, self.odom_phi]
            # self.estimation = [self.cam_x, self.cam_y, self.cam_phi]
            
            [self.input_v, self.input_omega] = self.go_to_goal()
            # [self.input_v, self.input_omega] = [5.0, 0]
        
        if(self.moving == 0):
            self.reset_goal()
            
    def measurement_fusion(self, k_filter_camera, k_filter_odo):
        # take the measurement from the odom
        odom_measurement = [self.odom_x, self.odom_y, self.odom_phi]
        
        cam_measurement = [self.cam_x, self.cam_y, self.cam_phi]
        
        
        k_filter_odo.R_k = np.array([[1.0,   0,    0],
                                     [  0, 1.0,    0],
                                     [  0,    0, 1.0]]) 
        k_filter_odo.Q_k = np.array([[0.01,   0,    0],
                                     [  0, 0.01,    0],
                                     [  0,    0, 0.005]]) 
        optimal_state_estimate_k, covariance_estimate_k = k_filter_odo.sr_EKF(odom_measurement, self.estimation, 1)
        # obs_vector_z_k = self.measurement_bias, # Most recent sensor measurement
        # state_estimate_k_1 = self.estimation, # Our most recent estimate of the state
        # u_k_1 = [v, omega], # Our most recent control input
        # P_k_1, # Our most recent state covariance matrix
        # dk = 1 # Time interval            
        self.measurement_Kalman = optimal_state_estimate_k
        k_filter_odo.P_k_1 = covariance_estimate_k
        self.estimation = self.measurement_Kalman
        
                
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

        # STORING VARIABLES
        # self.total_trips_abs = dict()
        # self.total_trips_rel = dict()
        self.saved_data = dict()

    def print_fuc(self):
        for tag in self.nodes:
            self.nodes[tag].print_position_measures()

    
    # def set_control_input(self, v = 1.5, omega = 0.0):
    #     # tag = '1'
    #     # [a, b] = self.nodes[tag].go_to_goal()
    #     self.msg_inputs.data = [0, 0]
    #     print("input: ", v)
    #     print("angle: ", omega)
    #     self.msg_input.linear.x = v
    #     self.msg_input.linear.y = 0.0
    #     self.msg_input.linear.z = 0.0

    #     self.msg_input.angular.x = omega
    #     self.msg_input.angular.y = 0.0
    #     self.msg_input.angular.z = 0.0
    #     self.publisher_input.publish(self.msg_input)
        
        
        # tag = '1'
        # print(self.msg_inputs.data)
        # print(np.array([self.nodes[tag].input_v, self.nodes[tag].input_omega]))
        # self.msg_inputs.data = np.concatenate((self.msg_inputs.data, np.array([self.nodes[tag].input_v, self.nodes[tag].input_omega])))
        # print(self.msg_inputs.data)
        # for tag in self.forager_tags:
        #     self.msg_inputs.data = np.concatenate((self.msg_inputs.data, np.array([self.nodes[tag].input_v, self.nodes[tag].input_omega])))
        # self.publisher_inputs.publish(self.msg_inputs)    
        
        

    # def stop_engine(self):
    #     self.msg_input.linear.x = 0.0
    #     self.msg_input.linear.y = 0.0
    #     self.msg_input.linear.z = 0.0

    #     self.msg_input.angular.x = 0.0
    #     self.msg_input.angular.y = 0.0
    #     self.msg_input.angular.z = 0.0
    #     self.publisher_input.publish(self.msg_input)

        
    # def loop_fuc(self):
    #     v = 0.0
    #     omega = 0.0
    #     for tag in self.nodes:
    #         self.nodes[tag].loop_fuc()
    #         if tag == '1':
    #             v = self.nodes[tag].input_v
    #             omega = self.nodes[tag].input_omega
                
    #     self.set_control_input(v, omega)

    def move(self, step_size: float = 0., theta: float =0.0):
        for tag in self.nodes:
            
            # could add a logic to let robor decide where to go           
            [step, omega] = self.nodes[tag].go_to_goal()
            step = step_size
            omega = theta
            
            self.nodes[tag].compute_move(pol = np.array([step, omega])) # \TODO change to move
            
            self.nodes[tag].estimation = self.nodes[tag].states_transform(self.nodes[tag].estimation, step, omega)
            print("tag: {}, estimation: {}".format(tag, self.nodes[tag].estimation))
            
            print("tag: {}, step: {}, theta: {}".format(tag, step, omega))
            
        # SETUP MESSAGE
        self.msg_auto_motive.data = np.array([len(self.nodes)])
        for tag in self.nodes:
            self.msg_auto_motive.data = np.concatenate((self.msg_auto_motive.data,
                    np.array([int(self.nodes[tag].address)]), self.nodes[tag].msg_auto_motive), axis=0)

        # SEND MOVE MESSAGE
        rospy.sleep(1)
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
    
    def reinitialize_locations(self, type="all"):
        # DUMP Json with measured information
        meas_dict = {tag: {'pos': list(self.nodes[tag].pos[-1]), 'orien':self.nodes[tag].orien[-1]} for tag in
                     self.forager_tags}
        with open('reinitialize.json','w') as outfile:
            json.dump(meas_dict, outfile)

        try:
            with open('reinitialize.json') as json_file:
                mapper = json.load(json_file)
        except:
            print('FOUT in REINITIALIZE')

        reinitialized_tags = []
        for tag in mapper:
            if tag in self.nodes:
                reinitialized_tags += [tag]
                if type=="all":
                    self.nodes[tag].pos[-1] = np.array(mapper[tag]['pos'])
                    self.nodes[tag].orien[-1] = mapper[tag]['orien']
                elif type=="pos":
                    self.nodes[tag].pos[-1] = np.array(mapper[tag]['pos'])
                elif type=="orien":
                    self.nodes[tag].orien[-1] = mapper[tag]['orien']
        return reinitialized_tags

    def print_position_measures(self):
        for tag in self.nodes:
            self.nodes[tag].print_position_measures()

    def store_data(self, t):
        self.saved_data[t] = dict()
        for tag in self.nodes:
            # self.saved_data[t][tag] = {'pos': copy.deepcopy(self.nodes[tag].pos),
            #                            'orien': copy.deepcopy(self.nodes[tag].orien),
            #                            'w': copy.deepcopy(self.nodes[tag].w),
            #                            'v': copy.deepcopy(self.nodes[tag].v),
            #                            'mode': copy.deepcopy(self.nodes[tag].mode),
            #                            'state': copy.deepcopy(self.nodes[tag].state),
            #                            'trips': copy.deepcopy(self.nodes[tag].trips)
            #                            }

            # self.saved_data[t][tag] = {'pos_x': copy.deepcopy(self.nodes[tag].pos[-1][0]),
            #                            'pos_y': copy.deepcopy(self.nodes[tag].pos[-1][1]),
            #                            'orien': copy.deepcopy(self.nodes[tag].orien[-1])
            #                            }
            self.saved_data[t][tag] = {'pos_x': copy.deepcopy(self.nodes[tag].robot_meas_pose[0]),
                                       'pos_y': copy.deepcopy(self.nodes[tag].robot_meas_pose[1]),
                                       'orien': copy.deepcopy(self.nodes[tag].robot_meas_orien),
                                       'x': copy.deepcopy(self.nodes[tag].estimation[0]),
                                       'y': copy.deepcopy(self.nodes[tag].estimation[1]),
                                       'phi': copy.deepcopy(self.nodes[tag].estimation[2]),
                                       'cam_x': copy.deepcopy(self.nodes[tag].cam_x),
                                       'cam_y': copy.deepcopy(self.nodes[tag].cam_y),
                                       'cam_phi': copy.deepcopy(self.nodes[tag].cam_phi)
                                       }


    def save_data(self, t):
        with open('./data/saved_data_t{}_RUN{}.p'.format(t, 1),'wb') as fp:
            pickle.dump(self.saved_data, fp, protocol=pickle.HIGHEST_PROTOCOL)





