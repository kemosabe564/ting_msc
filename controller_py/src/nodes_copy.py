from configuration import *

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


from sensor_msgs.msg import Imu



import rospy
import numpy as np
import functions
import time
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import copy
import pickle

# import tf_conversions
import tf2_ros

# for info on the parallel execution part, check Python Threading Tutorial: Run Code concurrently using the threading module
# import concurrent.futures

wheel_distance = 0.041
t_delay = 0.5
std_vlt_trans = 20
std_vlt_rot = 10

with open('mapper.json') as json_file:
    mapper = json.load(json_file)

obstacle_avoidance = functions.ObstacleAvoidance()

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
        self.robot_accel = np.NaN
        self.robot_input = np.NaN

        self.input_v = 0.0
        self.input_omega = 0.0

        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0

        # initialize theoretical positioning
        # \TODO Remember we initialize the first move as well
        self.pos = np.array([nest_location, self.start_pos, self.start_pos, self.start_pos, self.start_pos])
        self.orien = np.array([self.start_orien,self.start_orien,self.start_orien,self.start_orien,self.start_orien])

        # Initialize rostopic listeners/writers
        self.listener_robot_pose = rospy.Subscriber('swarm/elisa3_robot_{}/odom'.format(self.tag), Odometry,
                                                    self.listen_robot_pose_callback)
        
        self.listener_robot_accel = rospy.Subscriber('accel', Imu,
                                                    self.listen_robot_accel_callback)
        # /accel
        self.listener_robot_pose = rospy.Subscriber('mobile_base/cmd_vel', Twist, self.listen_robot_input_callback)

        # Initialize shared update message attributes
        self.update_leds = False
        self.msg_leds = np.array([0,0,0])
        self.update_auto_motive = False
        self.msg_auto_motive = np.array([0,0,0,0])
        self.update_reset = False
        self.msg_reset = np.array([0, 0, 0, 0])
        self.trigger_auto_motive = 103

        ##### SELF-GUIDED SWARM PARAMETERS #####
        self.range = range_of_influence
        self.state = [1,1]      # 0=beacon, 1=foraging
        self.mode = [0,0]         # 0=food-seeking (blue), 1=nest-seeking (red)

        self.neigh = []
        self.neigh_foragers = []
        self.neigh_beacons = []

        self.w = np.array([0., 0.])
        self.v = np.array([[0.,0.],[0.,0.]])
        self.move = np.array([[0.,0.],[0.,0.]])
        self.trips = 0
        self.taken_steps = 0

        self.calc_move = np.array([0.,0.])

        # \TODO Remember we initialize the first move as well
        self.move[-1] = self.pos[-1] - self.pos[0]
        self.led_state_0_mode_0()

    def listen_robot_accel_callback(self,accelMsg):
        # self.robot_meas_pose = np.round(np.array(1),3)
        # self.robot_meas_orien = np.round(float(1),3)
        self.robot_accel = accelMsg.linear_acceleration.x
        
    def listen_robot_pose_callback(self,odomMsg):
        self.robot_meas_pose = np.round(np.array([float(odomMsg.pose.pose.position.x),
                                                  float(odomMsg.pose.pose.position.y)]),3)
        self.robot_meas_orien = np.round(float(odomMsg.pose.pose.position.z),3)
        self.robot_meas_time = odomMsg.header.stamp.secs
        
    def listen_robot_input_callback(self, accelMsg):
            # self.robot_meas_pose = np.round(np.array(1),3)
            # self.robot_meas_orien = np.round(float(1),3)
            self.robot_input = accelMsg.linear.x
            print(self.robot_input)

    def publish_greenLed(self, intensity = np.array([0])):
        self.update_leds = True
        self.msg_leds[0] = intensity

    def publish_redLed(self, intensity = np.array([0])):
        self.update_leds = True
        self.msg_leds[1] = intensity

    def publish_blueLed(self, intensity = np.array([0])):
        self.update_leds = True
        self.msg_leds[2] = intensity

    def led_state_1(self):
        self.publish_greenLed(intensity=np.array([led_std_intensity]))
        self.publish_blueLed(intensity=np.array([0]))
        self.publish_redLed(intensity=np.array([0]))

    def led_state_0_mode_0(self):
        self.publish_greenLed(intensity=np.array([0]))
        self.publish_blueLed(intensity=np.array([led_std_intensity]))
        self.publish_redLed(intensity=np.array([0]))

    def led_state_0_mode_1(self):
        self.publish_greenLed(intensity=np.array([0]))
        self.publish_blueLed(intensity=np.array([0]))
        self.publish_redLed(intensity=np.array([led_std_intensity]))

    def led_off(self):
        self.publish_greenLed(intensity=np.array([0]))
        self.publish_blueLed(intensity=np.array([0]))
        self.publish_redLed(intensity=np.array([0]))

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

    def move_fnc(self, pol=np.array([0,0])): #global pol = [rho, phi]
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

        ## left -right = 0,  -left right = 1
        if delta_phi >= 0:
            self.msg_auto_motive = np.array([1, delta_phi, 0, pol[0]])
        else:
            self.msg_auto_motive = np.array([0, -delta_phi, 0, pol[0]])

        # if self.mode[1] == 0:
        #     self.msg_leds = np.array([0,0,90])
        # else:
        #     self.msg_leds = np.array([0,90,0])

        # self.update_leds = True
        self.update_leds = False

        self.update_auto_motive = True

        if self.trigger_auto_motive==103:
            self.trigger_auto_motive =104
        else:
            self.trigger_auto_motive =103

        
        
        # UPDATE POSITION
        # this is used for calculate how much to move further in cart ccordination
        # print("pol: ", pol)
        # print("phi_cor: ", phi_cor)
        prop_move_cart = functions.pol2cart(np.array([pol[0], phi_cor]))
        print("prop_move_cart: ", prop_move_cart)
        
        # print("self pos: ", self.pos)
        
        calc_point, obs_avoid_mode = obstacle_avoidance.obstacle_avoidance(self.pos[-1], prop_move_cart)

        print("calc_point: ", calc_point)
        
        print("old self pos: ", self.pos)
        
        # the renew_vec works like a FIFO buffer
        self.pos = functions.renew_vec(self.pos)
        self.orien = functions.renew_vec(self.orien)

        print("new self pos: ", self.pos)
        
        self.pos[-1] = calc_point
        self.orien[-1] += delta_phi

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
        # print("msg: ", msg)
        print("robot_meas_pose: ", self.robot_meas_pose)
        print("robot_meas_orien: ", self.robot_meas_orien)
        
        
        # print("accel: ", self.robot_accel)
        # print("input: ", self.robot_input)

    ##### SELF-GUIDED SWARM FUNCTIONS #####
    def neigh_agents(self, nodes):
        self.neigh = [tag for tag in nodes if np.linalg.norm(nodes[tag].pos[-1] - self.pos[-1]) < self.range and
                      tag != self.tag]
        self.neigh_foragers = [tag for tag in self.neigh if nodes[tag].state[1] == 1]
        self.neigh_beacons = [tag for tag in self.neigh if nodes[tag].state[1] == 0]

    def _search_food(self):
        self.mode[0] = self.mode[1]
        self.mode[1] = 0
        self.led_state_0_mode_0()

    def _search_nest(self):
        self.mode[0] = self.mode[1]
        self.mode[1] = 1
        self.led_state_0_mode_1()

    def in_range(self, location):
        return np.linalg.norm(self.pos[-1] - np.array(location)) <= target_range + numeric_step_margin

    def move_random(self):
        random_angle = self.orien[-1] + np.random.normal(loc=0,scale=random_move_std)
        return functions.pol2cart(np.array([step_size, random_angle])) # pol = [rho, phi]

    def _pick_direction(self, nodes):
        if self.mode[1]==0:
            w_type = 1
        else:
            w_type = 0

        if (self.mode[0]==0 and self.in_range(food_location)) or (self.mode[1]==1 and self.in_range(nest_location)):
            return -functions.normalize(self.move[1])*step_size

        weighted_vecs = [nodes[beac_tag].v[w_type]*nodes[beac_tag].w[w_type] for beac_tag in self.neigh_beacons]
        weights = [nodes[beac_tag].w[w_type] for beac_tag in self.neigh_beacons]

        if not weighted_vecs:
            return self.move_random()
        elif max(weights) < step_threshold:
            return self.move_random()
        elif exploration_rate>np.random.uniform(0,1):
            return self.move_random()

        weighted_vecs /= max(weights)
        vec = functions.normalize(sum(weighted_vecs))*step_size

        if np.isnan(vec).any() or np.linalg.norm(vec) <= step_threshold:
            print('generated vec in _pick direction contains NaN or is too small')
            return self.move_random()
        return vec

    def pre_step(self, nodes):
        self.calc_move = functions.cart2pol(self._pick_direction(nodes))

    def inter_step(self):
        if self.taken_steps==1:
            self.move_fnc(pol=np.array([0.,0.]))
            self.taken_steps += 1
        else:
            self.move_fnc(pol=self.calc_move)
            self.taken_steps+=1

    def after_step(self):
        self.move = functions.renew_vec(self.move)
        self.move[-1] = self.pos[-1] - self.pos[0]

        if self.mode[1]==0 and self.in_range(food_location):
            self.trips += 1
            self._search_nest()
        elif self.mode[1]==1 and self.in_range(nest_location):
            self.trips+=1
            self._search_food()
        else:
            self.mode[0] = self.mode[1]

class Nodes:
    def __init__(self, active_robots = ['0000']):
        for tag in active_robots:
            if tag not in mapper:
                print('mapper robot: ' + str(tag) +' is missing ')
                break

        # SETUP TO RELEASE
        self.active_robots = active_robots
        self.N_total = len(active_robots)
        if self.N_total % 2 == 0:
            self.tags_release_order = {-1: [self.active_robots[0]]}
            tags2release = list(self.active_robots[1:-1])[::-1]
            self.tags_release_order[0] = [self.active_robots[-1]]
            for i in range(0, int((self.N_total -1 )/ N_batch)):
                self.tags_release_order[i+1] = tags2release[((i) *2):((i+1)*2)]
        elif self.N_total % 2==1:
            self.tags_release_order = {-1: [self.active_robots[0]]}
            tags2release = list(self.active_robots[1:])[::-1]
            for i in range(0, int((self.N_total -1 )/ N_batch)):
                self.tags_release_order[i] = tags2release[((i) *2):((i+1)*2)]

        rospy.init_node('python_node')
        self.publisher_auto_motive = rospy.Publisher("elisa3_all_robots/auto_motive", Float64MultiArray,
                                                     queue_size=10)
        self.publisher_leds = rospy.Publisher("elisa3_all_robots/leds", Float64MultiArray,
                                                     queue_size=10)
        self.publisher_reset = rospy.Publisher("elisa3_all_robots/reset", Float64MultiArray,
                                              queue_size=10)
        
        self.publisher_input = rospy.Publisher("mobile_base/input", Twist,
                                              queue_size=10)
        
        self.msg_input = Twist()
        
        
        self.msg_auto_motive = Float64MultiArray()
        self.msg_leds = Float64MultiArray()
        self.msg_reset = Float64MultiArray()

        self.beac_tags = []
        self.forager_tags = []

        if DEBUG_ROBOTS:
            self.nodes = {tag: Node(release_time=0, tag=tag) for tag in active_robots}
            self.forager_tags = active_robots
        else:
            self.nodes = {}
            _ = self.release_robot(-1)

        # self.check_states()
        # self.switch_states()

        # STORING VARIABLES
        self.total_trips_abs = dict()
        self.total_trips_rel = dict()
        self.saved_data = dict()

    def print_fuc(self):
        if DEBUG_ROBOTS:
            for tag in self.forager_tags:
                self.nodes[tag].print_position_measures()

    def set_control_input(self, v = 1.0, omega = 0.0):
        self.msg_input.linear.x = v
        self.msg_input.linear.y = 0.0
        self.msg_input.linear.z = 0.0

        self.msg_input.angular.x = omega
        self.msg_input.angular.y = 0.0
        self.msg_input.angular.z = 0.0
        self.publisher_input.publish(self.msg_input)

    def stop_engine(self):
        # for tag in self.forager_tags:
        #     self.nodes[tag].stop_engine()
        self.msg_input.linear.x = 0.0
        self.msg_input.linear.y = 0.0
        self.msg_input.linear.z = 0.0

        self.msg_input.angular.x = 0.0
        self.msg_input.angular.y = 0.0
        self.msg_input.angular.z = 0.0
        self.publisher_input.publish(self.msg_input)
        
    def move_fnc(self):
        # with concurrent.futures.ThreadPoolExecutor() as executor:
        #     results = [executor.submit(self.nodes[tag].move, self.nodes[tag].move_random()) for tag in self.nodes]

        # IF DEBUGGING OVERRULE CREATE IND MESSAGES
        if DEBUG_ROBOTS:
            for tag in self.forager_tags:
                print("tag: ", tag)
                self.nodes[tag].move_fnc(pol=np.array([step_size, 0.1]))

        # SETUP MESSAGE
        self.msg_auto_motive.data = np.array([0])
        count = 0
        for tag in self.forager_tags:
            if self.nodes[tag].update_auto_motive:
                self.msg_auto_motive.data = np.concatenate((self.msg_auto_motive.data,
                        np.array([int(self.nodes[tag].address)]), self.nodes[tag].msg_auto_motive), axis=0)
                count += 1
        self.msg_auto_motive.data[0] = count

        # SEND MOVE MESSAGE
        if not DEBUG_NO_ROBOTS:
            rospy.sleep(1)
        self.publisher_auto_motive.publish(self.msg_auto_motive)

        if NEED_LEDS:
            # SETUP LED MESSAGE
            if not DEBUG_NO_ROBOTS:
                rospy.sleep(1)
            for tag in self.forager_tags:
                self.nodes[tag].publish_greenLed(intensity=np.array([self.nodes[tag].trigger_auto_motive]))

            # SEND LED MESSAGE
            self.update_leds(specified_tags=self.forager_tags)

            # CORRECT LED MESSAGE
            for tag in self.forager_tags:
                self.nodes[tag].publish_greenLed(intensity=np.array([0]))

    def update_leds(self, specified_tags=None):
        self.msg_leds.data = np.array([0])
        count = 0

        if specified_tags:
            ToIterateOver = specified_tags
        else:
            ToIterateOver = list(self.nodes.keys())

        for tag in ToIterateOver:
            if self.nodes[tag].update_leds:
                self.msg_leds.data = np.concatenate((self.msg_leds.data, np.array([int(self.nodes[tag].address)]),
                                 self.nodes[tag].msg_leds), axis=0)
                count += 1
        self.msg_leds.data[0] = count

        if not DEBUG_NO_ROBOTS:
            rospy.sleep(3)
        self.publisher_leds.publish(self.msg_leds)

    def reset(self, type = 'odom',specified_tags=None):
        self.msg_reset.data = np.array([0])
        count = 0

        if specified_tags:
            ToIterateOver = specified_tags
        else:
            ToIterateOver = list(self.nodes.keys())

        for tag in ToIterateOver:
            self.nodes[tag].reset(type=type)

            # ONLY REPLENISH MESSAGE IF THEOR UPDATE TYPE
            if self.nodes[tag].update_reset:
                self.msg_reset.data = np.concatenate((self.msg_reset.data, np.array([int(self.nodes[tag].address)]),
                                                      self.nodes[tag].msg_reset), axis=0)
                count +=1

        if count != 0:
            self.msg_reset.data[0] = count
            if not DEBUG_NO_ROBOTS:
                rospy.sleep(1)
            self.publisher_reset.publish(self.msg_reset)

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

    def print_position_measures(self, pre_specified_tags=None):
        if pre_specified_tags:
            for tag in pre_specified_tags:
                self.nodes[tag].print_position_measures()
        else:
            for tag in self.nodes:
                self.nodes[tag].print_position_measures()

    def turn_off_lights(self):
        for tag in self.nodes:
            self.nodes[tag].led_off()
        self.update_leds()

    ##### SELF-GUIDED SWARM FUNCTIONS #####
    def neigh_agents(self):
        for tag in self.nodes:
            self.nodes[tag].neigh_agents(self.nodes)

    def check_states(self):
        self.beac_tags = []
        self.forager_tags = []
        for tag in self.nodes:
            if self.nodes[tag].state[1] == 0:
                self.beac_tags += [tag]
            elif self.nodes[tag].state[1] == 1:
                self.forager_tags += [tag]

    def update_agents(self):
        self.check_states()
        self.neigh_agents()

    def pre_steps(self):
        for forager_tag in self.forager_tags:
            self.nodes[forager_tag].pre_step(self.nodes)

    def inter_steps(self):
        for forager_tag in self.forager_tags:
            self.nodes[forager_tag].inter_step()

    def after_steps(self):
        for forager_tag in self.forager_tags:
            self.nodes[forager_tag].after_step()

    def release_robot(self, step):
        if step in self.tags_release_order:
            for tag in self.tags_release_order[step]:
                self.nodes[tag] = Node(release_time=step, tag=tag)

            if not DEBUG_NO_ROBOTS:
                self.reset(type='theor', specified_tags=self.tags_release_order[step])
                rospy.sleep(5)

            self.print_position_measures()
            return self.tags_release_order[step]
        else:
            return None

    def switch_states(self):
        forager_tags_old = self.forager_tags.copy()
        beacon_tages_old = self.beac_tags.copy()
        tages_changed = []

        for tag in forager_tags_old:
            self.nodes[tag].state[0] = self.nodes[tag].state[1]

            if not self.nodes[tag].neigh_beacons:
                tages_changed += [tag]
                self.nodes[tag].state[1] = 0

                # self.update_beacon_w_v(tag,[tag]) \TODO ENABLE

                self.beac_tags += [tag]
                self.forager_tags.remove(tag)
                self.neigh_agents()

                self.nodes[tag].led_state_1()
                self.nodes[tag].update_auto_motive = False
                self.nodes[tag].update_reset = False


        # self.update_leds()

    def evaporate_weights(self):
        for beac_tag in self.beac_tags:
            self.nodes[beac_tag].w *= (1-rho)

    def reward(self, weights, rew):
        if rew:
            return rho*rew
        else:
            return rho * (lam * max(weights, default=0))

    def update_beacon_w_v(self, beac_tag, forager_tags):
        beac_v = np.array([[0.,0.],[0.,0.]])
        foragers_w_update = np.array([0.,0.])
        count_v0 = 0
        count_v1 = 0

        for forager_tag in forager_tags:
            W1_weights = [self.nodes[tag].w[0] for tag in self.nodes[forager_tag].neigh_beacons]
            W2_weights = [self.nodes[tag].w[1] for tag in self.nodes[forager_tag].neigh_beacons]

            forager_move = self.nodes[forager_tag].move[1]
            forager_w_update = np.array([0.,0.])

            if self.nodes[forager_tag].mode[0]==0:
                if self.nodes[forager_tag].in_range(nest_location):
                    forager_w_update[0] = self.reward(W1_weights, rew=rew)

                    count_v0 += 1
                    beac_v[0] += -forager_move
                elif self.nodes[forager_tag].in_range(food_location):
                    forager_w_update[1] = self.reward(W2_weights, rew=rew)
                else:
                    forager_w_update[0] = self.reward(W1_weights, rew=0)

                    count_v0 += 1
                    beac_v[0] += -forager_move

            elif self.nodes[forager_tag].mode[0]==1:
                if self.nodes[forager_tag].in_range(food_location):
                    forager_w_update[1] = self.reward(W2_weights, rew=rew)

                    count_v1 += 1
                    beac_v[1] += -forager_move
                elif self.nodes[forager_tag].in_range(nest_location):
                    forager_w_update[0] = self.reward(W1_weights, rew=rew)
                else:
                    forager_w_update[1] = self.reward(W2_weights, rew=0)

                    count_v1 += 1
                    beac_v[1] += -forager_move

            foragers_w_update += forager_w_update
            self.nodes[beac_tag].w[0] += forager_w_update[0] / len(forager_tags)
            self.nodes[beac_tag].w[1] += forager_w_update[1] / len(forager_tags)

        if np.linalg.norm(self.nodes[beac_tag].v[0]) and count_v0:
            self.nodes[beac_tag].v[0] *= (1 - rho_v)
            self.nodes[beac_tag].v[0] += rho_v * beac_v[0] / count_v0
        elif count_v0:
            self.nodes[beac_tag].v[0] += rho_v * beac_v[0] / count_v0

        if np.linalg.norm(self.nodes[beac_tag].v[1]) and count_v1:
            self.nodes[beac_tag].v[1] *= (1 - rho_v)
            self.nodes[beac_tag].v[1] += rho_v * beac_v[1] / count_v1
        elif count_v1:
            self.nodes[beac_tag].v[1] += rho_v * beac_v[1] / count_v1

        if self.nodes[beac_tag].in_range(nest_location):
            self.nodes[beac_tag].v[0] = np.array([0.,0.])
        if self.nodes[beac_tag].in_range(food_location):
            self.nodes[beac_tag].v[1] = np.array([0.,0.])

    def update_weights(self):
        for beac_tag in self.beac_tags:
            self.update_beacon_w_v(beac_tag, self.nodes[beac_tag].neigh_foragers)

    # REGULATION FUNCTIONS
    def sim_pre_step(self,t):
        # ACTION
        tags_just_released = []
        if len(self.nodes) < self.N_total:
            tags_just_released = self.release_robot(step=t)
        # self.update_leds()

        # UPDATE
        self.update_agents()
        if '0' in self.nodes and tags_just_released:
            self.update_beacon_w_v('0',tags_just_released)

        # ACTION
        self.pre_steps()

    def sim_inter_pre_step(self):
        # ACTION
        self.inter_steps()
        if not DEBUG_NO_ROBOTS:
            rospy.sleep(1)
        self.update_leds()      # \TODO TURN ON CAMERA!!
        if not DEBUG_NO_ROBOTS:
            rospy.sleep(3)
        self.move_fnc()
        if not DEBUG_NO_ROBOTS:
            rospy.sleep(10)

    def sim_inter_after_step(self):
        if not DEBUG_NO_ROBOTS:
            self.reset(type="odom")
        self.print_position_measures()

    def sim_after_step(self,t):
        if t % reinitialize_steps == 0 and not DEBUG_NO_ROBOTS: #\TODO add print all
            reinitialize_tags = self.reinitialize_locations(type="all")
            rospy.sleep(1)
            self.reset(type="theor") #\TODO alway check if reset has arived
            rospy.sleep(7)
            self.print_position_measures(pre_specified_tags=reinitialize_tags)

        # ACTION
        self.after_steps()

        # UPDATE
        self.update_agents()

        # ACTION
        self.switch_states()

        # ACTION
        self.evaporate_weights()
        self.update_weights()

        # UPDATE
        self.update_agents()

        # STORE
        self.store_nr_trips(t)
        self.store_data(t)

    def store_nr_trips(self,t):
        trips = [self.nodes[tag].trips for tag in self.nodes]
        self.total_trips_abs[t] = sum(trips)
        self.total_trips_rel[t] = sum(trips)/len(self.nodes)

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
                                       'orien': copy.deepcopy(self.nodes[tag].robot_meas_orien)
                                       }



    def save_data(self, t):
        with open('./data/saved_data_t{}_RUN{}.p'.format(t, run),'wb') as fp:
            pickle.dump(self.saved_data, fp, protocol=pickle.HIGHEST_PROTOCOL)



        # save data per time_step
        # with open('./stored_data/saved_data_t{}_RUN{}.p'.format(t,run),'wb') as fp:
        #     pickle.dump(self.saved_data, fp, protocol=pickle.HIGHEST_PROTOCOL)
        #
        # # save end data
        # saved_end_data = dict()
        # for tag in self.nodes:
        #     saved_end_data[tag] = {'pos': copy.deepcopy(self.nodes[tag].pos),
        #                            'orien': copy.deepcopy(self.nodes[tag].orien),
        #                            'w': copy.deepcopy(self.nodes[tag].w),
        #                            'v': copy.deepcopy(self.nodes[tag].v),
        #                            'mode': copy.deepcopy(self.nodes[tag].mode),
        #                            'state': copy.deepcopy(self.nodes[tag].state),
        #                            'trips': copy.deepcopy(self.nodes[tag].trips),
        #                            'neigh': copy.deepcopy(self.nodes[tag].neigh),
        #                            'neigh_foragers': copy.deepcopy(self.nodes[tag].neigh_foragers),
        #                            'neigh_beacons': copy.deepcopy(self.nodes[tag].neigh_beacons),
        #                            'sim_time': t,
        #                            'release_time': copy.deepcopy(self.nodes[tag].release_time),
        #                            'address': copy.deepcopy(self.nodes[tag].address),
        #                            'update_leds': copy.deepcopy(self.nodes[tag].update_leds),
        #                            'msg_leds': copy.deepcopy(self.nodes[tag].msg_leds),
        #                            'update_auto_motive': copy.deepcopy(self.nodes[tag].update_auto_motive),
        #                            'msg_auto_motive': copy.deepcopy(self.nodes[tag].msg_auto_motive),
        #                            'update_reset': copy.deepcopy(self.nodes[tag].update_reset),
        #                            'msg_reset': copy.deepcopy(self.nodes[tag].msg_reset),
        #                            'taken_steps': copy.deepcopy(self.nodes[tag].taken_steps),
        #                            'move': copy.deepcopy(self.nodes[tag].move),
        #                            'calc_move': copy.deepcopy(self.nodes[tag].calc_move)
        #                            }
        #
        # with open('./stored_data/saved_end_data_t{}_RUN{}.p'.format(t,run),'wb') as fp:
        #     pickle.dump(saved_end_data, fp, protocol=pickle.HIGHEST_PROTOCOL)


    def plot_field(self, type='theor', plot_text=False, fig_tag=0, save=True):
        if type=='theor':
            plt.figure(fig_tag*2)
        elif type=='odom':
            plt.figure(fig_tag*2+1)

        plt.plot([nest_location[0], food_location[0]],
                 [nest_location[1], food_location[1]], '*r')

        plt.xlim(domain[0][0], domain[2][0])
        plt.ylim(domain[0][1], domain[1][1])

        weights = {tag: sum(self.nodes[tag].w) for tag in self.nodes}

        if type=='theor':
            for beac_tag in self.beac_tags:
                if max(weights.values()) > step_threshold:
                    size = np.log(weights[beac_tag]/ max(weights.values())+1)*10
                else:
                    size = 1
                plt.plot([self.nodes[beac_tag].pos[-1][0]], [self.nodes[beac_tag].pos[-1][1]],
                         'o', color='green', markersize=size)
                if self.nodes[beac_tag].w[0] > step_threshold and np.linalg.norm(
                        self.nodes[beac_tag].v[0]) > step_threshold:
                    arrow0 = functions.normalize(self.nodes[beac_tag].v[0]) * step_size
                    plt.plot([self.nodes[beac_tag].pos[-1][0], self.nodes[beac_tag].pos[-1][0] + arrow0[0]],
                             [self.nodes[beac_tag].pos[-1][1], self.nodes[beac_tag].pos[-1][1] + arrow0[1]],
                             color='black')
                if self.nodes[beac_tag].w[1] > step_threshold and np.linalg.norm(
                        self.nodes[beac_tag].v[1]) > step_threshold:
                    arrow1 = functions.normalize(self.nodes[beac_tag].v[1]) * step_size
                    plt.plot([self.nodes[beac_tag].pos[-1][0], self.nodes[beac_tag].pos[-1][0] + arrow1[0]],
                             [self.nodes[beac_tag].pos[-1][1], self.nodes[beac_tag].pos[-1][1] + arrow1[1]],
                             color='blue')

                if plot_text:
                    plt.text(self.nodes[beac_tag].pos[-1][0], self.nodes[beac_tag].pos[-1][1],beac_tag)

            for for_tag in self.forager_tags:
                if self.nodes[for_tag].mode[1] ==0:
                    plt.plot([self.nodes[for_tag].pos[-1][0]], [self.nodes[for_tag].pos[-1][1]],
                         'b*', markersize=2)
                else:
                    plt.plot([self.nodes[for_tag].pos[-1][0]], [self.nodes[for_tag].pos[-1][1]],
                             'r*', markersize=2)
                if plot_text:
                    plt.text(self.nodes[for_tag].pos[-1][0], self.nodes[for_tag].pos[-1][1], for_tag)


        elif type=='odom':
            for beac_tag in self.beac_tags:
                if max(weights.values()) > step_threshold:
                    size = np.log(weights[beac_tag]/ max(weights.values())+1)*10
                else:
                    size = 1
                plt.plot([self.nodes[beac_tag].robot_meas_pose[0]], [self.nodes[beac_tag].robot_meas_pose[1]],
                         'o', color='black', markersize=size)

                if self.nodes[beac_tag].w[0] > step_threshold and np.linalg.norm(
                        self.nodes[beac_tag].v[0]) > step_threshold:
                    arrow0 = functions.normalize(self.nodes[beac_tag].v[0]) * step_size
                    plt.plot([self.nodes[beac_tag].robot_meas_pose[0],
                              self.nodes[beac_tag].robot_meas_pose[0] + arrow0[0]],
                             [self.nodes[beac_tag].robot_meas_pose[1],
                              self.nodes[beac_tag].robot_meas_pose[1] + arrow0[1]],
                             color='black')
                if self.nodes[beac_tag].w[1] > step_threshold and np.linalg.norm(
                        self.nodes[beac_tag].v[1]) > step_threshold:
                    arrow1 = functions.normalize(self.nodes[beac_tag].v[1]) * step_size
                    plt.plot([self.nodes[beac_tag].robot_meas_pose[0],
                              self.nodes[beac_tag].robot_meas_pose[0] + arrow1[0]],
                             [self.nodes[beac_tag].robot_meas_pose[1],
                              self.nodes[beac_tag].robot_meas_pose[1] + arrow1[1]],
                             color='blue')

                if plot_text:
                    plt.text(self.nodes[beac_tag].robot_meas_pose[0],
                             self.nodes[beac_tag].robot_meas_pose[1],beac_tag)

            for for_tag in self.forager_tags:
                if self.nodes[for_tag].mode[1] ==0:
                    plt.plot([self.nodes[for_tag].robot_meas_pose[0]],
                             [self.nodes[for_tag].robot_meas_pose[1]],'b*', markersize=2)
                else:
                    plt.plot([self.nodes[for_tag].robot_meas_pose[0]],
                             [self.nodes[for_tag].robot_meas_pose[1]], 'r*', markersize=2)
                if plot_text:
                    plt.text(self.nodes[for_tag].robot_meas_pose[0],
                             self.nodes[for_tag].robot_meas_pose[1], for_tag)

        if save:
            plt.savefig('./figures/plot_TAG{}_{}.png'.format(fig_tag, type))
            plt.close()
        else:
            plt.show()
            plt.close()


    def plot_trips(self, t, run, save=True):
        trips_sequence = np.array([self.total_trips_abs[time] for time in range(0,t)]) / self.N_total

        plt.plot(np.array(range(0,t)), trips_sequence, 'r')
        plt.xlabel('Time')
        plt.ylabel('#Trips / #Agents')

        if save:
            plt.savefig('total_trips_T{}_run{}.png'.format(t,run))
            plt.close()
        else:
            plt.show()
            plt.close()

    def load_states(self, load_time):
        with open('./stored_data/saved_end_data_t{}_RUN{}.p'.format(load_time, run), 'rb') as fp:
            saved_states = pickle.load(fp)

        for tag in saved_states:
            self.nodes[tag] = Node(release_time=saved_states[tag]['release_time'], tag=tag)
            self.nodes[tag].pos = saved_states[tag]['pos']
            self.nodes[tag].orien = saved_states[tag]['orien']
            self.nodes[tag].w = saved_states[tag]['w']
            self.nodes[tag].v = saved_states[tag]['v']
            self.nodes[tag].mode = saved_states[tag]['mode']
            self.nodes[tag].state = saved_states[tag]['state']
            self.nodes[tag].trips = saved_states[tag]['trips']
            self.nodes[tag].neigh = saved_states[tag]['neigh']
            self.nodes[tag].neigh_foragers = saved_states[tag]['neigh_foragers']
            self.nodes[tag].neigh_beacons = saved_states[tag]['neigh_beacons']
            self.nodes[tag].release_time = saved_states[tag]['release_time']
            self.nodes[tag].address = saved_states[tag]['address']
            self.nodes[tag].update_leds = saved_states[tag]['update_leds']
            self.nodes[tag].msg_leds = saved_states[tag]['msg_leds']
            self.nodes[tag].update_auto_motive = saved_states[tag]['update_auto_motive']
            self.nodes[tag].msg_auto_motive = saved_states[tag]['msg_auto_motive']
            self.nodes[tag].update_reset = saved_states[tag]['update_reset']
            self.nodes[tag].msg_reset = saved_states[tag]['msg_reset']
            self.nodes[tag].taken_steps = 0
            self.nodes[tag].move = saved_states[tag]['move']
            self.nodes[tag].calc_move = saved_states[tag]['calc_move']

        self.reset(type='theor', specified_tags=list(saved_states.keys()))
        rospy.sleep(5)
        self.print_position_measures()

        return saved_states[tag]['sim_time']
