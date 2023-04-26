#include <sstream>
#include <fstream>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <ros/ros.h>
#include "pc-side-elisa3-library/elisa3-lib.h"
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tgmath.h>

#include <string>


#define DEBUG_ROS_PARAMS 1
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_ODOMETRY 0
#define DEBUG_ACCELEROMETER 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_RANGE_SENSORS 0

#define SENSORS_NUM 4
#define ACCELEROMETER 0
#define FLOOR 1
#define PROXIMITY 2
#define MOTOR_POSITION 3

#define ACTUATORS_NUM 8 //used to be 4
#define MOTORS 0
// changed variables
#define GREEN_LEDS 1
#define RED_LEDS 2
#define BLUE_LEDS 3
#define GREEN_LEDS_MOVE 4
#define RED_LEDS_MOVE 5
#define BLUE_LEDS_MOVE 6
#define GREEN_LEDS_RESET 7

#define WHEEL_DISTANCE 0.041		// Distance between wheels in meters (axis length). -
#define ROBOT_RADIUS 0.025			// meters.

// new variables
#define NUMBER_OF_DISTANCE_SENSORS 8
#define SENSOR_VALUE_DETECTION_THRESHOLD 50

#define COLLISION_VLT 5

int Bx_sup[18][18] = {{0, -121, 78, -43, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {121, 0, 199, 78, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {-78, -199, 0, -121, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {43, -78, 121, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
int By_sup[18][18] = {{0, 77, 121, 198, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {-77, 0, 43, 121, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {-121, -43, 0, 77, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {-198, -121, -77, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

// Open file for writing
std::ofstream fw("/home/steven/catkin_ws/src/elisa3_node_cpp/src/data_run3.txt", std::ofstream::out);


int total_events = 0;
int error_sup = 10;
bool trigger_motive = false;
bool done = false;
double start_time = 0;
double start_comm_time = 0;
double comm_diff = 0;
double comm_diff_temp = 0;
int top_max = 30;
struct timeval time_now;
struct timeval comm_time;
struct timeval last_time;
bool enabledSensors[SENSORS_NUM];
bool changedActuators_0[ACTUATORS_NUM];
bool changedActuators_1[ACTUATORS_NUM];
double speedLeft = 0;
double speedRight = 0;
// CHANGED
unsigned char ledNum = 0, ledState = 0;
std::string baseTag;
std::string nodeName;

ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;

std::map<int, ros::Publisher> optiPublishers;

ros::Publisher accelPublisher;
sensor_msgs::Imu accelMsg;
ros::Publisher floorPublisher;
visualization_msgs::Marker floorMsg;

ros::Subscriber cmdVelSubscriber;
ros::Publisher odomPublisher[100];
ros::Subscriber AllResetSubscriber;

//get position from python -- aka Optitrack + filter
std::map<int, ros::Subscriber> AutoMotiveSubscribers;
ros::Subscriber AllAutoMotiveSubscriber;
//nav_msgs::Odometry odomMsg[100];

ros::Publisher speedPublisher[100];
ros::Publisher headingPublisher[100];


class Robot{
public:
    double speedLeft, speedRight;
    static const int greenLed =0, greenLed_move=0, greenLed_reset=0;
    static const int redLed =0, redLed_move=0;
    static const int blueLed =0, blueLed_move=0;
    int count, address;
    int robTurning;

    static const int auto_move_after_trigger = false;

//	std::string tag;
    int tag;
    bool changedActuators[ACTUATORS_NUM];

    int robTheta, robXPos, robYPos, xPos_temp, yPos_temp, robXPos_fixed, robYPos_fixed;

    double xPos, yPos, zPos, theta, xPos_filtered, yPos_filtered, theta_filtered, theta_filtered_prev, theta_estimate, theta_estimate_prev;
    double xPos_filtered_prev, yPos_filtered_prev;
    double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
    double deltaXCorr, deltaYCorr;
    double xPosCorr, yPosCorr;
    double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;
    float robSpeed;

    static const int trigger_delay = 0;

    double x_target, y_target;

    nav_msgs::Odometry odomMsg;

    std_msgs::Float64 speedMsg;
    std_msgs::Float64 headingMsg;

    geometry_msgs::TransformStamped odomTrans;
    geometry_msgs::TransformStamped odomTransInit;



    // Obstacle Avoidance
    unsigned int proxData[8];
    static const int move_type = 0;

    void updateSensorData();
    void updateRosInfo();
    void IIR_filter();

    static const bool reset = false;
};

void Robot::updateSensorData() {
    float speed_max = getUmax();
    robXPos = getOdomXpos(address);
    robYPos = getOdomYpos(address);
    robXPos_fixed = getXpos_fixed(address);
    robYPos_fixed = getYpos_fixed(address);
    robTheta = getOdomTheta(address);
    robTurning = getTurn(address);
    robSpeed = getSpeed(address)*5;

//        std::cout << "xPos " << tag << " :: [" << robXPos << ", " << robYPos << "], Events::" << getNbEvents(address) << ", " <<getNbResets(address) <<std::endl;
//        std::cout << "xPos_fixed:: " << tag << " :: [" << robXPos_fixed << ", " << robYPos_fixed << "]" << " " << getNbEvents(address) <<std::endl;
//        std::cout << "cam_pos:: " << tag << " :: [" << xPos_filtered*1000 << ", " << yPos_filtered*1000  << "]" <<std::endl;
//        std::cout << "cam_pos (k-1) :: " << tag << " :: [" << xPos_filtered_prev*1000 << ", " << yPos_filtered_prev*1000  << "]" <<std::endl;
//        std::cout << "theta:: " << tag << " :: [" << getOdomTheta_filtered(address) << ", " << robTheta  << "]" <<std::endl;
//        std::cout << "turn:: " << robTurning << std::endl;
//        std::cout << "time :: " << comm_diff << getAccZ(address) <<std::endl;
//////
       //std::cout << "" << std::endl;

        gettimeofday(&time_now, nullptr);
        if (((time_now.tv_sec*1000) + (time_now.tv_usec/1000))-((last_time.tv_sec*1000) + (last_time.tv_usec/1000)) > 5){
            fw << tag << ", " << ((time_now.tv_sec*1000) + (time_now.tv_usec/1000)) << ", " << robTheta << ", "<< getOdomTheta_filtered(address) << ", " << xPos_filtered*1000 << ", " << robXPos << ", " << yPos_filtered*1000 << ", " << robYPos << ", " << robTurning << ", "<< theta_filtered << ", "<< theta_estimate <<  ", "<< speed_max << ", " << getNbEvents(address) << ", " << getNbResets(address) << ", "<< comm_diff <<"\n";
            if (tag == 2){
                gettimeofday(&last_time, nullptr);
            }
        }

        if (robXPos_fixed == getOdomXpos_temp1(address)){
            gettimeofday(&comm_time, nullptr);
            start_comm_time = (comm_time.tv_sec*1000) + (comm_time.tv_usec/1000);
        }else{
            gettimeofday(&comm_time, nullptr);
            comm_diff_temp = (comm_time.tv_sec*1000) + (comm_time.tv_usec/1000) - start_comm_time;
            if (comm_diff_temp > comm_diff){
                comm_diff = comm_diff_temp;
            }
        }
}

std::map<int, Robot> robots_dict;
//std::map<std::string, Robot> robots_dict; ////

void Robot::updateRosInfo() {
    // just sending odometry data to rviz -- also include publish to odomPublisher (?)

    xPos = robXPos/ 1000.0;
    yPos = robYPos/ 1000.0;
    theta = robTheta*M_PI/180;    


    // Publish message over ROS
    odomMsg.header.stamp = ros::Time::now();
    std::stringstream ss;
    ss << "elisa3_robot_" << tag;
    odomMsg.header.frame_id = ss.str();
    std::stringstream ss2;
    ss2 << "elisa3_robot_" << tag <<"/base_link";
    odomMsg.child_frame_id = ss2.str();
    odomMsg.pose.pose.position.x = xPos;
    odomMsg.pose.pose.position.y = yPos;
    odomMsg.pose.pose.position.z = 0;
    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
    odomMsg.pose.pose.orientation = odomQuat;


    odomPublisher[tag].publish(odomMsg);

    // broadcast odom over tf -- line 498
    //Header
    odomTrans.header.stamp = ros::Time::now();
    odomTrans.header.frame_id =  ss.str();
    odomTrans.child_frame_id = ss2.str();
    // Main msg
    odomTrans.header.stamp = ros::Time::now();
    odomTrans.transform.translation.x = xPos;
    odomTrans.transform.translation.y = yPos;
    odomTrans.transform.translation.z = 0.0;

    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(theta);
    odomTrans.transform.rotation = odomQuat_trans;

    // broadcast odomInit over tf
    odomTransInit.header.stamp = ros::Time::now();


    //send speed and heading
    if (robTurning == 1){
        speedMsg.data = 0;
    }else{
        speedMsg.data = robSpeed;
    }

    speedPublisher[tag].publish(speedMsg);

    headingMsg.data = theta_filtered*M_PI/180;

    headingPublisher[tag].publish(headingMsg);
}

void Robot::IIR_filter(){
  int c1 = 6, c2 = 8;

  theta_filtered = c1*theta_filtered_prev + theta_estimate + theta_estimate_prev;
  theta_filtered = theta_filtered/c2;

  theta_filtered_prev = theta_filtered;
  theta_estimate_prev = theta_estimate;

}


void updateSensorsData() {
    std::map<int, Robot>::iterator it;

    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        robots_dict[it->first].updateSensorData();
    }
}

void updateRosInfo() {
    static tf2_ros::TransformBroadcaster br; // handles broadcasting of coordinate data

    std::map<int, Robot>::iterator it;
    // iterates over all the robots and updates the ros of the robot using different function
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        // This updates the odomTrans and odomInit data in the robot struct
        robots_dict[it->first].updateRosInfo();

        //Send the data to tf topic
        br.sendTransform(robots_dict[it->first].odomTrans);
        br.sendTransform(robots_dict[it->first].odomTransInit);
    }
}

// New line of code --
int getIdFromAddress(int address) {
    std::map<int, Robot>::iterator it;
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        if(robots_dict[it->first].address == address) {
            return int(it->first);
        }
    }
    return -1;
}

void handlerVelocity(const geometry_msgs::Twist::ConstPtr& msg) {
    // Controls the velocity of each wheel based on linear and angular velocities.
    double linear = msg->linear.x;		// Expect the linear speed to be given in cm/s.
    double angular = msg->angular.z;	// Expect the angular speed to be given in rad/s.

    // if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name[0] << "] " << "linear: " << linear << ", angular: " << angular << std::endl;

    // Kinematic model for differential robot.
    double wl = (linear - ((WHEEL_DISTANCE*100.0) / 2.0) * angular);	// Result is cm/s.
    double wr = (linear + ((WHEEL_DISTANCE*100.0) / 2.0) * angular);	// Result is cm/s.

    //if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name[0] << "] " << "kinematic: " << wl << ", " << wr << std::endl;

    // At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
    speedLeft = int(wl * 10.0 / 5.0);		// Transform the speed to mm/s and then divide by 5 to get the correct value to send to the elisa robot,
    speedRight = int(wr * 10.0 / 5.0);	// that is 1/5 of mm/s (a value of 10 means 50 mm/s).


    if(speedLeft > 127) {
        speedLeft = 127;
    }
    if(speedLeft < -127) {
        speedLeft = -127;
    }
    if(speedRight > 127) {
        speedRight = 127;
    }
    if(speedRight < -127) {
        speedRight = -127;
    }

    // update all robots with the same speed
    std::map<int, Robot>::iterator it;
    total_events = 0;
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        robots_dict[it->first].speedLeft = speedLeft;
        robots_dict[it->first].speedRight = speedRight;
        robots_dict[it->first].changedActuators[MOTORS] = true;

        if (speedLeft > 0 or speedRight > 0){
            enableObstacleAvoidance(robots_dict[it->first].address);
            gettimeofday(&time_now, nullptr);
            start_time = (time_now.tv_sec*1000) + (time_now.tv_usec/1000);
        }
        total_events += getNbEvents(robots_dict[it->first].address) - getNbResets(robots_dict[it->first].address);
    }

    if ((speedLeft<0 and speedRight<0)){
        float speed_max = getUmax();

        if(fw.is_open()){
            gettimeofday(&time_now, nullptr);

            //fw << ((time_now.tv_sec*1000) + (time_now.tv_usec/1000)) - start_time << ", " << speed_max << ", " << total_events << "\n";

            //std::cout <<"Data saved to file"<<std::endl;

        }
    }
}

void handlerAllAutoMove(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int nr_robots = int(msg->data[0]);
    int tag;
    double disX, disY;
    double angle;

    XmlRpc::XmlRpcValue::iterator i;
    for (int i=0; i < nr_robots; i++){
        tag = getIdFromAddress(int(msg->data[i*4+1]));

        robots_dict[tag].xPos_filtered_prev = robots_dict[tag].xPos_filtered;
        robots_dict[tag].yPos_filtered_prev = robots_dict[tag].yPos_filtered;

        // PER ROBOT HANDLING (either xPos or robXPos) --
        robots_dict[tag].xPos_filtered = double(msg->data[i*4+2]);
        robots_dict[tag].yPos_filtered = double(msg->data[i*4+3]);

        //Adjust theta based on x and y displacement
        if(robots_dict[tag].robTurning == 0){
            disX = -robots_dict[tag].xPos_filtered_prev+robots_dict[tag].xPos_filtered;
            disY = -robots_dict[tag].yPos_filtered_prev+robots_dict[tag].yPos_filtered;
            angle = atan2(disY, disX);

            if ((disX>0.001 || disY>0.001 || disX<-0.001 || disY<-0.001)){ //fabs(angle)<M_PI/2 && --
                robots_dict[tag].theta_estimate = angle*57.3;
                std::cout << "Angle calculated: " << robots_dict[tag].theta_estimate << "\n";
            }else{
                robots_dict[tag].theta_estimate = robots_dict[tag].theta_estimate;
                std::cout << "Angle reused: " << robots_dict[tag].theta_filtered << "\n";
            }

            // IIR filter -- remove large peaks
            //robots_dict[tag].IIR_filter();

        }else{
        // robot is turning -- estimate is wrong use odometry theta
            robots_dict[tag].theta_estimate = robots_dict[tag].robTheta;
            robots_dict[tag].theta_filtered = robots_dict[tag].robTheta;
        }

    }

    done = true;
    for (int i=0; i < nr_robots; i++){
        for (int j=0; j < nr_robots; j++){
            if(!((fabs(robots_dict[i].xPos_filtered - robots_dict[j].xPos_filtered) < fabs(Bx_sup[j][i]) + error_sup and fabs(robots_dict[i].xPos_filtered - robots_dict[j].xPos_filtered) > fabs(Bx_sup[j][i]) - error_sup) and (fabs(robots_dict[i].yPos_filtered - robots_dict[j].yPos_filtered) < fabs(By_sup[j][i]) + error_sup and fabs(robots_dict[i].yPos_filtered - robots_dict[j].yPos_filtered) > fabs(By_sup[j][i]) - error_sup))){
                done = false;
            }
        }
    }
    
    
}

void handlerAllReset(const std_msgs::Float64MultiArray::ConstPtr& msg) {

    std::map<int, Robot>::iterator it;

    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
          setXpos(robots_dict[it->first].address, robots_dict[it->first].xPos_filtered*1000); //update live position of the robots
          setYpos(robots_dict[it->first].address, robots_dict[it->first].yPos_filtered*1000);
          setTheta(robots_dict[it->first].address, robots_dict[it->first].theta_filtered*10);
    }
    std::cout << "Reset msg" << std::endl;
    enableReset();
    resetTheta();
}

bool * get_sensors_condition(unsigned int proxData[8]){
    static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = {false};

    for (int i=0; i < NUMBER_OF_DISTANCE_SENSORS; i++){
        if(proxData[i] > SENSOR_VALUE_DETECTION_THRESHOLD){
            sensors_condition[i] = true;
        } else {
            sensors_condition[i] = false;
        }
    }
    return sensors_condition;
}

void updateActuators() {
    // needs to be changed to send xpos and ypos ----

    char buff[6];

    std::map<int, Robot>::iterator it;

    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        bool *is_sensor_active = get_sensors_condition(robots_dict[it->first].proxData);

        if(robots_dict[it->first].changedActuators[MOTORS]){
            robots_dict[it->first].changedActuators[MOTORS] = false;

            setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft); //update speed of the robots -- used for start-up
            setRightSpeed(robots_dict[it->first].address, robots_dict[it->first].speedRight);

            if (getInitFlag()){
                setXpos(robots_dict[it->first].address, robots_dict[it->first].xPos_filtered*1000); //update live position of the robots
                setYpos(robots_dict[it->first].address, robots_dict[it->first].yPos_filtered*1000); //TODO change to x/yPos_filtered

                setXpos_fixed(robots_dict[it->first].address, robots_dict[it->first].robXPos_fixed); //update broadcast pos of the robots
                setYpos_fixed(robots_dict[it->first].address, robots_dict[it->first].robYPos_fixed);

                setTheta(robots_dict[it->first].address, robots_dict[it->first].theta_filtered*10); // in degrees
            }

        }

    }

}


int main(int argc,char *argv[]) {
    unsigned char sensorsEnabled = 0;
    int i = 0;

    gettimeofday(&last_time, nullptr);
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other -
    * part of the ROS system.
    */
    ros::init(argc, argv, "elisa3_node_cpp");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle np("~"); // Private.
    ros::NodeHandle n; // Public.

    np.param<std::string>("base_tag", baseTag, "elisa3");
    np.param<std::string>("name", nodeName, "elisa3");

    // Remote procedure call -- is a protocol used to communicate between programs or ask for a request
    XmlRpc::XmlRpcValue body_list;
    np.param("rigid_bodies", body_list, body_list); // refers to the file mocap.yaml with all the id's and robot addresses

    int N_robots = body_list.size();
    int robot_addresses[N_robots];

    if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0){
        XmlRpc::XmlRpcValue::iterator i;

        int count = 0;
        // iterate over all the robot entries in the yaml file and assign data to the robot cla
        for (i = body_list.begin(); i != body_list.end(); ++i) {
//		    std::cout << "[" << nodeName << "] " << "init number:: " << count << std::endl;
            Robot class_inst;
            class_inst.speedLeft =0;
            class_inst.speedRight = 0;
            class_inst.robTurning = 0;
            //class_inst.greenLed=0;

            class_inst.count = count;

            class_inst.address = (i->second)["address"];


            class_inst.xPos = (i->second)["xPos"];
            class_inst.yPos = (i->second)["yPos"];
            class_inst.xPos_filtered = class_inst.xPos;
            class_inst.yPos_filtered = class_inst.yPos;
            class_inst.robXPos_fixed = class_inst.xPos*1000;
            class_inst.robYPos_fixed = class_inst.yPos*1000;
            class_inst.xPos_filtered_prev = class_inst.xPos;
            class_inst.yPos_filtered_prev = class_inst.yPos;
            class_inst.theta = (i->second)["theta"];
            class_inst.theta_filtered = class_inst.theta;

            class_inst.changedActuators[MOTORS] = false;
            class_inst.tag = atoi((i->first).c_str()); // tag is ID of robot

            class_inst.odomTrans.header.frame_id  = "elisa3_robot_"+ boost::to_string(class_inst.tag);
            class_inst.odomTrans.child_frame_id = "elisa3_robot_"+ boost::to_string(class_inst.tag);

            // Geometry msgs -- used for what (?)
            class_inst.odomTransInit.header.frame_id  = "world";
            class_inst.odomTransInit.child_frame_id = "elisa3_robot_"+ boost::to_string(class_inst.tag);
            class_inst.odomTransInit.header.stamp = ros::Time::now();
            class_inst.odomTransInit.transform.translation.x = 0.0;
            class_inst.odomTransInit.transform.translation.y = 0.0;
            class_inst.odomTransInit.transform.translation.z = 0.0;
            geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(0);
            class_inst.odomTransInit.transform.rotation = odomQuat_trans;

            // robots_dict is a list of pointers to all the robots + variables
            robots_dict[class_inst.tag] = class_inst;

            robot_addresses[count] = class_inst.address;

            std::stringstream ss;
            ss << "elisa3_robot_" << class_inst.tag <<"/odom";
            odomPublisher[class_inst.tag] = np.advertise<nav_msgs::Odometry>(ss.str(), 10);

            std::stringstream ss1;
            ss1 << "elisa3_robot_" << class_inst.tag <<"/speed";
            speedPublisher[class_inst.tag] = np.advertise<std_msgs::Float64>(ss1.str(), 10);

            std::stringstream ss2;
            ss2 << "elisa3_robot_" << class_inst.tag <<"/heading";
            headingPublisher[class_inst.tag] = np.advertise<std_msgs::Float64>(ss2.str(), 10);

            disableObstacleAvoidance(class_inst.address);

            // Topic that get position + theta estimation from Optitrack
            AllAutoMotiveSubscriber = n.subscribe("elisa3_all_robots/auto_motive", 10, handlerAllAutoMove);

            AllResetSubscriber = n.subscribe("elisa3_all_robots/reset", 10, handlerAllReset);

            count += 1;

        }
    }
    np.param("accelerometer", enabledSensors[ACCELEROMETER], false);
    np.param("floor", enabledSensors[FLOOR], false);
    np.param("proximity", enabledSensors[PROXIMITY], false);
    np.param("motor_position", enabledSensors[MOTOR_POSITION], false);

    startCommunication(robot_addresses, N_robots);

    // Enable obstacle avoidance for all robots
    std::map<int, Robot>::iterator it;
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        disableObstacleAvoidance(robots_dict[it->first].address);

        setXpos(robots_dict[it->first].address, robots_dict[it->first].xPos_filtered*1000);
        setYpos(robots_dict[it->first].address, robots_dict[it->first].yPos_filtered*1000);

        setXpos_fixed(robots_dict[it->first].address, robots_dict[it->first].xPos*1000);
        setYpos_fixed(robots_dict[it->first].address, robots_dict[it->first].yPos*1000);

        setTheta(robots_dict[it->first].address, robots_dict[it->first].theta*573); // in radians
    }

    cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, handlerVelocity);


    //updateRosInfo();
    while (ros::ok()) {
        updateSensorsData();
        updateRosInfo();
        updateActuators();
        ros::spinOnce();

    }
    stopCommunication();
}