
#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "elisa3-lib.h"
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>

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

#define WHEEL_DISTANCE 0.041		// Distance between wheels in meters (axis length).
#define ROBOT_RADIUS 0.025			// meters.

// new variables
#define NUMBER_OF_DISTANCE_SENSORS 8
#define SENSOR_VALUE_DETECTION_THRESHOLD 50

#define COLLISION_VLT 5

bool trigger_motive = false;

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
//nav_msgs::Odometry odomMsg[100];


class Robot{
public:
    double speedLeft, speedRight;
    static const int greenLed =0, greenLed_move=0, greenLed_reset=0;
    static const int redLed =0, redLed_move=0;
    static const int blueLed =0, blueLed_move=0;
    int count, address;

    static const int auto_move_after_trigger = false;

//	std::string tag;
    int tag;
    bool changedActuators[ACTUATORS_NUM];

    int robTheta, robXPos, robYPos;

    double xPos, yPos, zPos, theta;
    double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
    double deltaXCorr, deltaYCorr;
    double xPosCorr, yPosCorr;
    double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;

    static const int trigger_delay = 0;

    double x_target, y_target;

	nav_msgs::Odometry odomMsg;

    geometry_msgs::TransformStamped odomTrans;
    geometry_msgs::TransformStamped odomTransInit;

    // Obstacle Avoidance
    unsigned int proxData[8];
    static const int move_type = 0;

    void updateSensorData();
    void updateRosInfo();

    static const bool reset = false;
};

void Robot::updateSensorData() {
    robXPos = getOdomXpos(address);
    robYPos = getOdomYpos(address);
    robTheta = getOdomTheta(address);
//	if(tag == 0){
//		std::cout << "[" << nodeName << "] " << "robIdx:: " << tag << ", robXPos:: " << robXPos << std::endl;
//	}

//    getAllProximity(address, proxData);
}

std::map<int, Robot> robots_dict;
//std::map<std::string, Robot> robots_dict;

void Robot::updateRosInfo() {
    // just sending odometry data to rviz -- also include publish to odomPublisher (?)

	robDeltaX = robXPos - robXPosPrev;
	robDeltaY = robYPos - robYPosPrev;
	robXPosPrev = robXPos;
	robYPosPrev = robYPos;				
	theta = robTheta*M_PI/180;    // Expressed in radiant.
	// We noticed from field tests on a vertical wall that there is a difference in the measured distance between
	// a route traveled toward bottom and a route traveled toward top. For this reason we adjust the distance 
	// traveled based on the angle.
//	if(tag == 1){
//		std::cout << "[" << nodeName << "] " << "robIdx:: " << tag << ", robDeltaX:: " << robDeltaX << std::endl;
//	}

	if(robTheta <= 180 && robTheta >= 0) {
			robDistTraveled = sqrt(robDeltaX*robDeltaX + robDeltaY*robDeltaY);
			deltaXCorr = robDistTraveled*2/3*cos(theta);	// 2/3 is the magical factor found from filed tests, probably you'll need to adapt it to your surface.
			deltaYCorr = robDistTraveled*2/3*sin(theta);
			//if(DEBUG_ODOMETRY)std::cout << "[" << elisa3Name << "] " << "delta corr: " << deltaXCorr << ", " << deltaYCorr << std::endl;
	} else {
			deltaXCorr = robDeltaX;
			deltaYCorr = robDeltaY;
			//if(DEBUG_ODOMETRY)std::cout << "[" << elisa3Name << "] " << "delta not corr: " << deltaXCorr << ", " << deltaYCorr << std::endl;
	}
	xPos += deltaXCorr/1000.0;	// Expressed in meters.
	yPos += deltaYCorr/1000.0;	// Expressed in meters.
	// if(tag == 1){
    //     std::cout << "[" << nodeName << "] " << "robIdx:: " << tag << ", xPos:: " << xPos << ", yPos:: " << yPos << std::endl;
    //     std::cout << "robXPos:: " << robXPos << ", robYPos:: " << robYPos << std::endl;
	// }
//	std::cout << "xPos" << ":: " << xPos << std::endl;
//	std::cout << "yPos" << ":: " << yPos << std::endl;


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
    odomMsg.pose.pose.position.z = 0.0;
    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
    odomMsg.pose.pose.orientation = odomQuat;
    //currentTime = ros::Time::now();
    //lastTime = ros::Time::now();

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

// New line of code
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
    
    std::cout << "speedRight:: " << speedRight << std::endl;
    std::cout << "speedLeft:: " << speedLeft << std::endl;


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
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        robots_dict[it->first].speedLeft = speedLeft;
        robots_dict[it->first].speedRight = speedRight;
        robots_dict[it->first].changedActuators[MOTORS] = true;
    }
    //changedActuators[MOTORS] = true;

    //if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name[0] << "] " << "new speed: " << speedLeft << ", " << speedRight << std::endl;

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
    // needs to be changed to send xpos and ypos

    char buff[6];

    std::map<int, Robot>::iterator it;

    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        bool *is_sensor_active = get_sensors_condition(robots_dict[it->first].proxData);

        if(robots_dict[it->first].changedActuators[MOTORS]){
            robots_dict[it->first].changedActuators[MOTORS] = false;

            setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft); //update speed of the robots
            setRightSpeed(robots_dict[it->first].address, robots_dict[it->first].speedRight);
			//std::cout << "Actuate agents:: " << robots_dict[it->first].address << "with::" << robots_dict[it->first].speedLeft << " and " << robots_dict[it->first].speedRight <<std::endl;
        }

        if(robots_dict[it->first].changedActuators[GREEN_LEDS_RESET]){
           robots_dict[it->first].changedActuators[GREEN_LEDS_RESET] = false;
           setGreen(robots_dict[it->first].address, robots_dict[it->first].greenLed_reset);
        }
    }

}


int main(int argc,char *argv[]) {
    unsigned char sensorsEnabled = 0;
    int i = 0;

    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
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
		    std::cout << "[" << nodeName << "] " << "init number:: " << count << std::endl;
            Robot class_inst;
            class_inst.speedLeft =0;
            class_inst.speedRight = 0;
            //class_inst.greenLed=0;

            class_inst.count = count;
            class_inst.address = (i->second)["address"];

            class_inst.xPos = (i->second)["xPos"];
            class_inst.yPos = (i->second)["yPos"];
            class_inst.theta = 0.0;
			//std::cout << "xPos" << ":: " << class_inst.xPos << std::endl;
			//std::cout << "yPos" << ":: " << class_inst.yPos << std::endl;

            class_inst.changedActuators[MOTORS] = false;
            class_inst.tag = atoi((i->first).c_str()); // tag is ID of robot

            class_inst.odomTrans.header.frame_id  = "elisa3_robot_"+ boost::to_string(class_inst.tag);
            class_inst.odomTrans.child_frame_id = "elisa3_robot_"+ boost::to_string(class_inst.tag);

            // Geometry msgs -- used for what (?)
            class_inst.odomTransInit.header.frame_id  = "world";
            class_inst.odomTransInit.child_frame_id = "elisa3_robot_"+ boost::to_string(class_inst.tag);
            class_inst.odomTransInit.header.stamp = ros::Time::now();
            class_inst.odomTransInit.transform.translation.x = class_inst.xPos;
            class_inst.odomTransInit.transform.translation.y = class_inst.yPos;
            class_inst.odomTransInit.transform.translation.z = 0.0;
            geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(class_inst.theta);
            class_inst.odomTransInit.transform.rotation = odomQuat_trans;

            // robots_dict is a list of pointers to all the robots + variables
            robots_dict[class_inst.tag] = class_inst;

            robot_addresses[count] = class_inst.address;

            // Why subscribe N times to the same topic (?)
            // AllLedSubscriber = n.subscribe("elisa3_all_robots/leds", 10, handlerAllLeds);

            //AllResetSubscriber = n.subscribe("elisa3_all_robots/reset", 10, handlerAllReset);

            //AllAutoMotiveSubscriber = n.subscribe("elisa3_all_robots/auto_motive", 10, handlerAllAutoMove);

			std::stringstream ss;
    		ss << "elisa3_robot_" << class_inst.tag <<"/odom";
			odomPublisher[class_inst.tag] = np.advertise<nav_msgs::Odometry>(ss.str(), 10);

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
        enableObstacleAvoidance(robots_dict[it->first].address);
    }

   cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, handlerVelocity);
    // cmdVelSubscriber = n.subscribe("mobile_base/input", 10, handlerVelocity);

	//updateRosInfo();
    while (ros::ok()) {
        updateSensorsData();
        updateRosInfo();
        updateActuators();
        ros::spinOnce();

		//for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        //    if (waitForUpdate(robots_dict[it->first].address, 10000000)) { // Wait for at most 10 seconds.
		//		std::cout << "Robot" << ":: " << robots_dict[it->first].address << " does not respond" << std::endl;
                //break; // We have connection problems, stop here.
        //    }
        //}
    }
    stopCommunication();
}



