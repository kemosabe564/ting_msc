import copy
import rospy
import pickle
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# for optitrack camera
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose




class Recorder:
    def __init__(self, tag):
        self.tag = tag
        self.odom_x = 0
        self.odom_y = 0
        self.odom_phi = 0
        self.odom_timer = 0
        self.cam_x = 0
        self.cam_y = 0
        self.cam_phi = 0
        self.cam_timer = 0
        
        self.listener_camera = rospy.Subscriber('Bebop{}/ground_pose'.format(self.tag + 1), Pose2D, self.listen_optitrack_callback)
        self.listener_camera_timer = rospy.Subscriber('Bebop{}/pose'.format(self.tag + 1), Pose, self.listen_optitrack_timer_callback)
        
        self.listener_robot_pose = rospy.Subscriber('elisa3_robot_{}/odom'.format(self.tag), Odometry,
                                                    self.listen_robot_pose_callback) 
        
    # functions for subscribe callback    
    def listen_optitrack_callback(self, optiMsg):
        self.cam_x = optiMsg.x
        self.cam_y = -optiMsg.y
        self.cam_phi = optiMsg.theta
        
    def listen_optitrack_timer_callback(self, optiMsg):
        self.timer = optiMsg.header.stamp.nsecs
        
    def listen_robot_pose_callback(self, odomMsg):
        self.odom_timer = odomMsg.header.stamp.secs
        self.odom_x = float(odomMsg.pose.pose.position.x)
        self.odom_y = float(odomMsg.pose.pose.position.y)
        self.odom_phi = float(odomMsg.pose.pose.position.z)  
                


class Recorders:
    def __init__(self, N) -> None:
        self.N = N
        self.saved_data = dict()
        self.recorders = {tag: Recorder(tag = tag) for tag in range(self.N)}
        self.measurement_list = np.zeros([self.N, 8])
        
    def update_measurement(self):
        self.measurement_list = np.zeros([self.N, 8])
        i = 0
        for tag in self.recorders:
            self.measurement_list[i][0] = self.recorders[tag].cam_x
            self.measurement_list[i][1] = self.recorders[tag].cam_y
            self.measurement_list[i][2] = self.recorders[tag].cam_phi
            self.measurement_list[i][3] = self.recorders[tag].timer
            self.measurement_list[i][4] = self.recorders[tag].odom_x
            self.measurement_list[i][5] = self.recorders[tag].odom_y
            self.measurement_list[i][6] = self.recorders[tag].odom_phi
            self.measurement_list[i][7] = self.recorders[tag].odom_timer
            
            i += 1                     
 
        
    def store_data(self, t):
        self.saved_data[t] = dict()
        for tag in self.recorders:
            self.saved_data[t][tag] = { 'pos_x': copy.deepcopy(self.recorders[tag].odom_x),
                                        'pos_y': copy.deepcopy(self.recorders[tag].odom_y),
                                        'orien': copy.deepcopy(self.recorders[tag].odom_phi),
                                        'cam_x': copy.deepcopy(self.recorders[tag].cam_x),
                                        'cam_y': copy.deepcopy(self.recorders[tag].cam_y),
                                        'cam_phi': copy.deepcopy(self.recorders[tag].cam_phi),
                                        'odom_timer': copy.deepcopy(self.recorders[tag].odom_timer),
                                        'cam_timer': copy.deepcopy(self.recorders[tag].cam_timer)
                                        }


    def save_data(self, t):
        with open('./data/all_data_t{}_RUN{}.p'.format(t, 1),'wb') as fp:
            pickle.dump(self.saved_data, fp, protocol=pickle.HIGHEST_PROTOCOL)

class TAG:
    def __init__(self) -> None:
        self.tag = "run"
        self.tag_subscriber = rospy.Subscriber('elisa3_tag', String, self.lisnter)
    
    def lisnter(self, textMsg):
        self.tag = textMsg.data
        print(self.tag)
        

if __name__ == "__main__":
    print("start") 
    recoders = Recorders(3)
    t = 0
    Tag = TAG()
    while(1):
    
        print("t: ", t)
        t += 1
        recoders.store_data(t)
        rospy.sleep(0.001)
        
        # if (Tag.tag == "stop"):
        #     break
        if(t > 4e4):
            break

    recoders.save_data(0)  
    
    print(t)
    print("done") 