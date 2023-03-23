
// #include <sstream>
// #include <math.h>
// #include <time.h>
// #include <sys/time.h>
// #include "elisa3-lib.h"
// #include <ros/ros.h>
// #include <sensor_msgs/Range.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/Imu.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Twist.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <tf/transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

// #include <visualization_msgs/Marker.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// //#include <opencv/cv.h>
// #include <opencv2/opencv.hpp>
// #include <sensor_msgs/LaserScan.h>

// #include <string>

// #define DEBUG_ROS_PARAMS 1
// #define DEBUG_UPDATE_SENSORS_DATA 0
// #define DEBUG_ODOMETRY 0
// #define DEBUG_ACCELEROMETER 0
// #define DEBUG_SPEED_RECEIVED 0
// #define DEBUG_RANGE_SENSORS 0

// #define SENSORS_NUM 4
// #define ACCELEROMETER 0
// #define FLOOR 1
// #define PROXIMITY 2
// #define MOTOR_POSITION 3

// #define ACTUATORS_NUM 8 //used to be 4
// #define MOTORS 0
// #define GREEN_LEDS 1
// #define RED_LEDS 2
// #define BLUE_LEDS 3
// #define GREEN_LEDS_MOVE 4
// #define RED_LEDS_MOVE 5
// #define BLUE_LEDS_MOVE 6
// #define GREEN_LEDS_RESET 7

// #define WHEEL_DISTANCE 0.041		// Distance between wheels in meters (axis length).
// #define ROBOT_RADIUS 0.025			// meters.

// #define NUMBER_OF_DISTANCE_SENSORS 8
// #define SENSOR_VALUE_DETECTION_THRESHOLD 50

// #define COLLISION_VLT 5

// int NOTHING = 0;
// int FORWARD = 1;
// int BACKWARD = 2;

// bool trigger_motive = false;

// int robotAddress[2];

// bool enabledSensors[SENSORS_NUM];
// bool changedActuators_0[ACTUATORS_NUM];
// bool changedActuators_1[ACTUATORS_NUM];

// // double speedLeft = 0;
// // double speedRight = 0;

// // CHANGED
// unsigned char ledNum = 0, ledState = 0;
// std::string baseTag;
// std::string nodeName;

// ros::Publisher proxPublisher[8];
// sensor_msgs::Range proxMsg[8];
// ros::Publisher laserPublisher;
// sensor_msgs::LaserScan laserMsg;

// std::map<int, ros::Publisher> optiPublishers;

// ros::Publisher accelPublisher;
// sensor_msgs::Imu accelMsg;
// ros::Publisher floorPublisher;
// visualization_msgs::Marker floorMsg;

// // ros::Subscriber cmdVelSubscriber;
// // ros::Publisher odomPublisher[100];
// //nav_msgs::Odometry odomMsg[100];

// std::map<int, ros::Subscriber> VelSubscribers;

// std::map<int, ros::Subscriber> GreenLedSubscribers;
// std::map<int, ros::Subscriber> RedLedSubscribers;
// std::map<int, ros::Subscriber> BlueLedSubscribers;
// ros::Subscriber AllLedSubscriber;

// std::map<int, ros::Subscriber> TargetSubscribers;
// std::map<int, ros::Subscriber> ResetSubscribers;
// ros::Subscriber AllResetSubscriber;
// std::map<int, ros::Subscriber> OptiSubscribers;
// std::map<int, ros::Subscriber> AutoMotiveSubscribers;
// ros::Subscriber AllAutoMotiveSubscriber;

// class Robot{
// public:
//     double speedLeft, speedRight;
//     int greenLed =0, greenLed_move=0, greenLed_reset=0;
//     int redLed =0, redLed_move=0;
//     int blueLed =0, blueLed_move=0;
//     int count, address;

//     int auto_move_after_trigger = false;

// //	std::string tag;
//     int tag;
//     bool changedActuators[ACTUATORS_NUM];

//     int robTheta, robXPos, robYPos;

//     double xPos, yPos, zPos, theta;
//     double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
//     double deltaXCorr, deltaYCorr;
//     double xPosCorr, yPosCorr;
//     double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;

//     int trigger_delay = 0;

//     double x_target, y_target;

// 	nav_msgs::Odometry odomMsg;

//     geometry_msgs::TransformStamped odomTrans;
//     geometry_msgs::TransformStamped odomTransInit;

//     // Obstacle Avoidance
//     unsigned int proxData[8];
//     int move_type = 0;

//     void updateSensorData();
//     void updateRosInfo();

//     bool reset = false;
// };

// void Robot::updateSensorData() {
//     robXPos = getOdomXpos(address);
//     robYPos = getOdomYpos(address);
//     robTheta = getOdomTheta(address);
// //    getAllProximity(address, proxData);
// }

// std::map<int, Robot> robots_dict;
// //std::map<std::string, Robot> robots_dict;

// void Robot::updateRosInfo() {
//     // just sending odometry data to rviz -- also include publish to odomPublisher (?)

// // 	robDeltaX = robXPos - robXPosPrev;
// // 	robDeltaY = robYPos - robYPosPrev;
// // 	robXPosPrev = robXPos;
// // 	robYPosPrev = robYPos;				
// // 	theta = robTheta*M_PI/180;    // Expressed in radiant.
// // 	// We noticed from field tests on a vertical wall that there is a difference in the measured distance between
// // 	// a route traveled toward bottom and a route traveled toward top. For this reason we adjust the distance 
// // 	// traveled based on the angle.

// // 	if(robTheta <= 180 && robTheta >= 0) {
// // 			robDistTraveled = sqrt(robDeltaX*robDeltaX + robDeltaY*robDeltaY);
// // 			deltaXCorr = robDistTraveled*2/3*cos(theta);	// 2/3 is the magical factor found from filed tests, probably you'll need to adapt it to your surface.
// // 			deltaYCorr = robDistTraveled*2/3*sin(theta);
// // 	} else {
// // 			deltaXCorr = robDeltaX;
// // 			deltaYCorr = robDeltaY;
// // 	}
// // 	xPos += deltaXCorr/1000.0;	// Expressed in meters.
// // 	yPos += deltaYCorr/1000.0;	// Expressed in meters.
// // 	if(tag == 1){
// //         std::cout << "[" << nodeName << "] " << "robIdx:: " << tag << ", xPos:: " << xPos << ", yPos:: " << yPos << std::endl;
// //         std::cout << "robXPos:: " << robXPos << ", robYPos:: " << robYPos << std::endl;
// // 	}
// // //	std::cout << "xPos" << ":: " << xPos << std::endl;
// // //	std::cout << "yPos" << ":: " << yPos << std::endl;


// //     // Publish message over ROS
// //     odomMsg.header.stamp = ros::Time::now();
// //     std::stringstream ss;
// //     ss << "elisa3_robot_" << tag;
// //     odomMsg.header.frame_id = ss.str();
// //     std::stringstream ss2;
// //     ss2 << "elisa3_robot_" << tag <<"/base_link";
// //     odomMsg.child_frame_id = ss2.str();
// //     odomMsg.pose.pose.position.x = xPos;
// //     odomMsg.pose.pose.position.y = yPos;
// //     odomMsg.pose.pose.position.z = 0.0;
// //     // Since all odometry is 6DOF we'll need a quaternion created from yaw.
// //     geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
// //     odomMsg.pose.pose.orientation = odomQuat;
// //     //currentTime = ros::Time::now();
// //     //lastTime = ros::Time::now();

// //     odomPublisher[tag].publish(odomMsg);

// //     // broadcast odom over tf -- line 498
// //     //Header
// //     odomTrans.header.stamp = ros::Time::now();
// //     odomTrans.header.frame_id =  ss.str();
// //     odomTrans.child_frame_id = ss2.str();

//     xPos = robXPos/ 1000.0;
//     yPos = robYPos/ 1000.0;
//     theta = robTheta * M_PI / 180;
    
//     // Main msg
//     odomTrans.header.stamp = ros::Time::now();
//     odomTrans.transform.translation.x = xPos;
//     odomTrans.transform.translation.y = yPos;
//     odomTrans.transform.translation.z = 0.0;

//     // Since all odometry is 6DOF we'll need a quaternion created from yaw.
//     geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(theta);
//     odomTrans.transform.rotation = odomQuat_trans;

//     // broadcast odomInit over tf
//     odomTransInit.header.stamp = ros::Time::now();
// }

// void updateSensorsData() {
//     std::map<int, Robot>::iterator it;

//     for (it = robots_dict.begin(); it != robots_dict.end(); it++){
//         robots_dict[it->first].updateSensorData();
//     }
// }

// void updateRosInfo() {
//     static tf2_ros::TransformBroadcaster br; // handles broadcasting of coordinate data

//     std::map<int, Robot>::iterator it;
//     // iterates over all the robots and updates the ros of the robot using different function
//     for (it = robots_dict.begin(); it != robots_dict.end(); it++){
//         // This updates the odomTrans and odomInit data in the robot struct
//         robots_dict[it->first].updateRosInfo();

//         //Send the data to tf topic
//         br.sendTransform(robots_dict[it->first].odomTrans);
//         br.sendTransform(robots_dict[it->first].odomTransInit);
//     }
// }

// int getIdFromAddress(int address) {
//     std::map<int, Robot>::iterator it;
//     for (it = robots_dict.begin(); it != robots_dict.end(); it++){
//         if(robots_dict[it->first].address == address) {
//             return int(it->first);
//         }
//     }
//     return -1;
// }

// void handlerAllReset(const std_msgs::Float64MultiArray::ConstPtr& msg) {
//     int nr_robots = int(msg->data[0]);
// //    std::cout << "[" << nodeName << "] " << "nr robots resetting: " << nr_robots << std::endl;
//     int tag;

//     XmlRpc::XmlRpcValue::iterator i;
//     for (int i=0; i < nr_robots; i++){
//         tag = getIdFromAddress(int(msg->data[i*5+1]));

//         double type;
//         type = double(msg->data[i*5+2]);

//         robots_dict[tag].odomTransInit.header.stamp = ros::Time::now();
//         if (type == 0){
//             robots_dict[tag].odomTransInit.transform.translation.x = double(msg->data[i*5+3]);
//             robots_dict[tag].odomTransInit.transform.translation.y = double(msg->data[i*5+4]);
//             robots_dict[tag].odomTransInit.transform.translation.z = 0.0;
//             geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(double(msg->data[i*5+5]));
//             robots_dict[tag].odomTransInit.transform.rotation = odomQuat_trans;

//             robots_dict[tag].changedActuators[GREEN_LEDS_RESET] = true;
//             if (robots_dict[tag].greenLed_reset ==101) {
//                 robots_dict[tag].greenLed_reset = 102;
//             } else {
//                 robots_dict[tag].greenLed_reset = 101;
//             }
//             robots_dict[tag].reset = true;
//         }
//     }
// }

// // OLD FUNCTION, HANDLING PER ROBOT
// //void handlerAutoMove(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
// //    int turn_type = int(msg->data[0]);
// //    int trans_type = int(msg->data[2]);
// //    if (turn_type==0){
// //        int prop_msg = int(double(double(msg->data[1])*(75.0/(2.0*M_PI))+100.0));
// //        if (prop_msg >= 175) {
// //            prop_msg = 175;
// //        }
// //        robots_dict[tag].redLed_move = prop_msg;
// //    } else if (turn_type==1){
// //        int prop_msg = int(double(double(msg->data[1])*(75.0/(2.0*M_PI))+175.0));
// //        if (prop_msg >= 250) {
// //            prop_msg = 250;
// //        } else if (prop_msg == 175){
// //            prop_msg = 100;
// //        }
// //        robots_dict[tag].redLed_move = prop_msg;
// //    } else {
// //        robots_dict[tag].redLed_move = 100;
// //    }
// //
// //    if (trans_type==0){
// //        int prop_msg = int(double(msg->data[3])*25 + 100);
// //        if (prop_msg >= 175){
// //            prop_msg = 175;
// //        }
// //        robots_dict[tag].blueLed_move = prop_msg;
// //    } else {
// //        int prop_msg = int(double(msg->data[3])*25 + 175);
// //        if (prop_msg >= 250){
// //            prop_msg = 250;
// //        } else if (prop_msg = 175){
// //            prop_msg = 100;
// //        }
// //        robots_dict[tag].blueLed_move = prop_msg;
// //    }
// //
// //    if (robots_dict[tag].greenLed_move ==103) {
// //        robots_dict[tag].greenLed_move = 104;
// //    } else {
// //        robots_dict[tag].greenLed_move = 103;
// //    }
// //    robots_dict[tag].changedActuators[GREEN_LEDS_MOVE] = true;
// //    robots_dict[tag].changedActuators[RED_LEDS_MOVE] = true;
// //    robots_dict[tag].changedActuators[BLUE_LEDS_MOVE] = true;
// //    std::cout << "[" << nodeName << "] " << "[robot " << tag <<
// //            "]" << "handler green led: " << robots_dict[tag].greenLed_move  << std::endl;
// //    std::cout << "[" << nodeName << "] " << "[robot " << tag <<
// //        "]" << "handler red led: " << robots_dict[tag].redLed_move  << std::endl;
// //    std::cout << "[" << nodeName << "] " << "[robot " << tag <<
// //        "]" << "handler blue led: " << robots_dict[tag].blueLed_move  << std::endl;
// //}

// void handlerAllAutoMove(const std_msgs::Float64MultiArray::ConstPtr& msg) {
//     int nr_robots = int(msg->data[0]);
// //    std::cout << "[" << nodeName << "] " << "nr robots: " << nr_robots << std::endl;
//     int tag;

//     XmlRpc::XmlRpcValue::iterator i;
//     for (int i=0; i < nr_robots; i++){
//         tag = getIdFromAddress(int(msg->data[i*5+1]));
// //        std::cout << "[" << nodeName << "] " << "tag " << tag << std::endl;
// //        std::cout << "[" << nodeName << "] " << "received msg " << msg << std::endl;

//         // PER ROBOT HANDLING
//         int turn_type = int(msg->data[i*5+2]);
//         int trans_type = int(msg->data[i*5+4]);
//         if (turn_type==0){
//             int prop_msg = int(double(double(msg->data[i*5+3])*(75.0/(2.0*M_PI))+100.0));
//             if (prop_msg >= 175) {
//                 prop_msg = 175;
//             }
//             robots_dict[tag].redLed_move = prop_msg;
//         } else if (turn_type==1){
//             int prop_msg = int(double(double(msg->data[i*5+3])*(75.0/(2.0*M_PI))+175.0));
//             if (prop_msg >= 250) {
//                 prop_msg = 250;
//             } else if (prop_msg == 175){
//                 prop_msg = 100;
//             }
//             robots_dict[tag].redLed_move = prop_msg;
//         } else {
//             robots_dict[tag].redLed_move = 100;
//         }

//         if (trans_type==0){
//             int prop_msg = int(double(msg->data[i*5+5])*25 + 100);
//             if (prop_msg >= 175){
//                 prop_msg = 175;
//             }
//             robots_dict[tag].blueLed_move = prop_msg;
//         } else {
//             int prop_msg = int(double(msg->data[i*5+5])*25 + 175);
//             if (prop_msg >= 250){
//                 prop_msg = 250;
//             } else if (prop_msg = 175){
//                 prop_msg = 100;
//             }
//             robots_dict[tag].blueLed_move = prop_msg;
//         }

//         if (robots_dict[tag].greenLed_move ==103) {
//             robots_dict[tag].greenLed_move = 104;
//         } else {
//             robots_dict[tag].greenLed_move = 103;
//         }
//         robots_dict[tag].changedActuators[GREEN_LEDS_MOVE] = true;
//         robots_dict[tag].changedActuators[RED_LEDS_MOVE] = true;
//         robots_dict[tag].changedActuators[BLUE_LEDS_MOVE] = true;
//     }
// }

// void handlerAllLeds(const std_msgs::Float64MultiArray::ConstPtr& msg) {
//     int nr_robots;
//     nr_robots = int(msg->data[0]);
//     int tag;

//     XmlRpc::XmlRpcValue::iterator i;
//     for (int i=0; i < nr_robots; i++){
//         tag = getIdFromAddress(int(msg->data[i*4+1]));
//         robots_dict[tag].changedActuators[GREEN_LEDS] = true;
//         robots_dict[tag].changedActuators[RED_LEDS] = true;
//         robots_dict[tag].changedActuators[BLUE_LEDS] = true;
//         robots_dict[tag].greenLed = int(msg->data[i*4+2]);
//         robots_dict[tag].redLed = int(msg->data[i*4+3]);
//         robots_dict[tag].blueLed = int(msg->data[i*4+4]);
//    }


// }

// // void handlerVelocity(const geometry_msgs::Twist::ConstPtr& msg) {
// //     // Controls the velocity of each wheel based on linear and angular velocities.
// //     double linear = msg->linear.x;		// Expect the linear speed to be given in cm/s.
// //     double angular = msg->angular.z;	// Expect the angular speed to be given in rad/s.

// //     // if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name[0] << "] " << "linear: " << linear << ", angular: " << angular << std::endl;

// //     // Kinematic model for differential robot.
// //     double wl = (linear - ((WHEEL_DISTANCE*100.0) / 2.0) * angular);	// Result is cm/s.
// //     double wr = (linear + ((WHEEL_DISTANCE*100.0) / 2.0) * angular);	// Result is cm/s.

// //     //if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name[0] << "] " << "kinematic: " << wl << ", " << wr << std::endl;

// //     // At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
// //     speedLeft = int(wl * 10.0 / 5.0);		// Transform the speed to mm/s and then divide by 5 to get the correct value to send to the elisa robot,
// //     speedRight = int(wr * 10.0 / 5.0);	// that is 1/5 of mm/s (a value of 10 means 50 mm/s).
    
// //     std::cout << "speedRight:: " << speedRight << std::endl;
// //     std::cout << "speedLeft:: " << speedLeft << std::endl;


// //     if(speedLeft > 127) {
// //         speedLeft = 127;
// //     }
// //     if(speedLeft < -127) {
// //         speedLeft = -127;
// //     }
// //     if(speedRight > 127) {
// //         speedRight = 127;
// //     }
// //     if(speedRight < -127) {
// //         speedRight = -127;
// //     }

// //     // update all robots with the same speed
// //     std::map<int, Robot>::iterator it;
// //     for (it = robots_dict.begin(); it != robots_dict.end(); it++){
// //         robots_dict[it->first].speedLeft = speedLeft;
// //         robots_dict[it->first].speedRight = speedRight;
// //         robots_dict[it->first].changedActuators[MOTORS] = true;
// //     }
// //     //changedActuators[MOTORS] = true;

// //     //if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name[0] << "] " << "new speed: " << speedLeft << ", " << speedRight << std::endl;

// // }

// bool * get_sensors_condition(unsigned int proxData[8]){
//     static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = {false};

//     for (int i=0; i < NUMBER_OF_DISTANCE_SENSORS; i++){
//         if(proxData[i] > SENSOR_VALUE_DETECTION_THRESHOLD){
//             sensors_condition[i] = true;
//         } else {
//             sensors_condition[i] = false;
//         }
//     }
//     return sensors_condition;
// }

// void updateActuators() {
    
//     char buff[6];

// 	std::map<int, Robot>::iterator it;

// 	for (it = robots_dict.begin(); it != robots_dict.end(); it++){
// 	    bool *is_sensor_active = get_sensors_condition(robots_dict[it->first].proxData);

//         if(robots_dict[it->first].changedActuators[MOTORS]){
//             robots_dict[it->first].changedActuators[MOTORS] = false;

//             setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft);
//             setRightSpeed(robots_dict[it->first].address, robots_dict[it->first].speedRight);
// 		}

//         if(robots_dict[it->first].changedActuators[BLUE_LEDS] and
//         robots_dict[it->first].changedActuators[RED_LEDS] and
//         robots_dict[it->first].changedActuators[GREEN_LEDS]) {
//             robots_dict[it->first].changedActuators[RED_LEDS] = false;
//             robots_dict[it->first].changedActuators[BLUE_LEDS] = false;
//             robots_dict[it->first].changedActuators[GREEN_LEDS] = false;
//             //            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
//             //            "]" << "updated blue leds move: " << robots_dict[it->first].blueLed_move << std::endl;
//             setAllColors(robots_dict[it->first].address, robots_dict[it->first].redLed,
//                          robots_dict[it->first].greenLed, robots_dict[it->first].blueLed);
//         }

//         if(robots_dict[it->first].changedActuators[BLUE_LEDS_MOVE] and
//             robots_dict[it->first].changedActuators[RED_LEDS_MOVE] and
//             robots_dict[it->first].changedActuators[GREEN_LEDS_MOVE]) {
//             robots_dict[it->first].changedActuators[RED_LEDS_MOVE] = false;
//             robots_dict[it->first].changedActuators[BLUE_LEDS_MOVE] = false;
//             robots_dict[it->first].changedActuators[GREEN_LEDS_MOVE] = false;
//             //            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
//             //            "]" << "updated blue leds move: " << robots_dict[it->first].blueLed_move << std::endl;
//             setAllColors(robots_dict[it->first].address, robots_dict[it->first].redLed_move,
//                          robots_dict[it->first].greenLed_move, robots_dict[it->first].blueLed_move);

//             trigger_motive = true;

//         }

//         // CHECK GREEN LED RESET
//         if(robots_dict[it->first].changedActuators[GREEN_LEDS_RESET]){
//             robots_dict[it->first].changedActuators[GREEN_LEDS_RESET] = false;
// //            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
// //            "]" << "updated green leds reset: " << robots_dict[it->first].greenLed_reset << std::endl;
//             setGreen(robots_dict[it->first].address, robots_dict[it->first].greenLed_reset);
//         }
// 	}
// }


// int main(int argc,char *argv[]) {
//     unsigned char sensorsEnabled = 0;
//     int i = 0;

//     /**
//     * The ros::init() function needs to see argc and argv so that it can perform
//     * any ROS arguments and name remapping that were provided at the command line.
//     * For programmatic remappings you can use a different version of init() which takes
//     * remappings directly, but for most command-line programs, passing argc and argv is
//     * the easiest way to do it.  The third argument to init() is the name of the node.
//     *
//     * You must call one of the versions of ros::init() before using any other
//     * part of the ROS system.
//     */
//     ros::init(argc, argv, "elisa3_node_cpp");

//     /**
//     * NodeHandle is the main access point to communications with the ROS system.
//     * The first NodeHandle constructed will fully initialize this node, and the last
//     * NodeHandle destructed will close down the node.
//     */
//     ros::NodeHandle np("~"); // Private.
//     ros::NodeHandle n; // Public.

//     np.param<std::string>("base_tag", baseTag, "elisa3");
//     np.param<std::string>("name", nodeName, "elisa3");

//     // Remote procedure call -- is a protocol used to communicate between programs or ask for a request
//     XmlRpc::XmlRpcValue body_list;
//     np.param("rigid_bodies", body_list, body_list); // refers to the file mocap.yaml with all the id's and robot addresses

//     int N_robots = body_list.size();
//     int robot_addresses[N_robots];

//     // initial

//     if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0){
//         XmlRpc::XmlRpcValue::iterator i;

//         int count = 0;
//         // iterate over all the robot entries in the yaml file and assign data to the robot cla
//         for (i = body_list.begin(); i != body_list.end(); ++i) {
// 		    std::cout << "[" << nodeName << "] " << "init number:: " << count << std::endl;
//             Robot class_inst;
//             class_inst.speedLeft =0;
//             class_inst.speedRight = 0;
//             class_inst.greenLed=0;

//             class_inst.count = count;
//             class_inst.address = (i->second)["address"];

//             // class_inst.xPos = (i->second)["xPos"];
//             // class_inst.yPos = (i->second)["yPos"];
//             // class_inst.theta = 0.0;
//             class_inst.xPos = getOdomXpos(class_inst.address);
//             class_inst.yPos = getOdomYpos(class_inst.address);
//             class_inst.theta = getOdomTheta(class_inst.address);

// 			//std::cout << "xPos" << ":: " << class_inst.xPos << std::endl;
// 			//std::cout << "yPos" << ":: " << class_inst.yPos << std::endl;

//             class_inst.changedActuators[MOTORS] = false;
//             class_inst.tag = atoi((i->first).c_str());

//             class_inst.odomTrans.header.frame_id  = "elisa3_robot_"+ std::to_string(class_inst.tag) + "_init";
//             class_inst.odomTrans.child_frame_id = "elisa3_robot_"+ std::to_string(class_inst.tag);

//             class_inst.odomTransInit.header.frame_id  = "world";
//             class_inst.odomTransInit.child_frame_id = "elisa3_robot_"+ std::to_string(class_inst.tag) + "_init";
//             class_inst.odomTransInit.header.stamp = ros::Time::now();
//             class_inst.odomTransInit.transform.translation.x = class_inst.xPos;
//             class_inst.odomTransInit.transform.translation.y = class_inst.yPos;
//             class_inst.odomTransInit.transform.translation.z = 0.0;
//             geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(class_inst.theta);
//             class_inst.odomTransInit.transform.rotation = odomQuat_trans;

//             // robots_dict is a list of pointers to all the robots + variables
//             robots_dict[class_inst.tag] = class_inst;

//             robotAddress[count] = class_inst.address;
//             robot_addresses[count] = class_inst.address;

//             AllLedSubscriber = n.subscribe("elisa3_all_robots/leds", 10, handlerAllLeds);

//             AllResetSubscriber = n.subscribe("elisa3_all_robots/reset", 10, handlerAllReset);

//             AllAutoMotiveSubscriber = n.subscribe("elisa3_all_robots/auto_motive", 10, handlerAllAutoMove);

// 			// std::stringstream ss;
//     		// ss << "elisa3_robot_" << class_inst.tag <<"/odom";
// 			// odomPublisher[class_inst.tag] = np.advertise<nav_msgs::Odometry>(ss.str(), 10);

//             count += 1;

//         }
//     }
//     np.param("accelerometer", enabledSensors[ACCELEROMETER], false);
//     np.param("floor", enabledSensors[FLOOR], false);
//     np.param("proximity", enabledSensors[PROXIMITY], false);
//     np.param("motor_position", enabledSensors[MOTOR_POSITION], false);

//     startCommunication(robot_addresses, N_robots);

//     // Enable obstacle avoidance for all robots
//     std::map<int, Robot>::iterator it;
//     for (it = robots_dict.begin(); it != robots_dict.end(); it++){
//         enableObstacleAvoidance(robots_dict[it->first].address);
//     }

// //    cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, handlerVelocity);

//     // cmdVelSubscriber = n.subscribe("mobile_base/input", 10, handlerVelocity);

// 	//updateRosInfo();
//     while (ros::ok()) {
//         updateSensorsData();
//         updateRosInfo();
//         updateActuators();
//         ros::spinOnce();

// 		//for (it = robots_dict.begin(); it != robots_dict.end(); it++){
//         //    if (waitForUpdate(robots_dict[it->first].address, 10000000)) { // Wait for at most 10 seconds.
// 		//		std::cout << "Robot" << ":: " << robots_dict[it->first].address << " does not respond" << std::endl;
//                 //break; // We have connection problems, stop here.
//         //    }
//         //}
//     }
//     stopCommunication();
// }


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

#define ACTUATORS_NUM 8
#define MOTORS 0
#define GREEN_LEDS 1
#define RED_LEDS 2
#define BLUE_LEDS 3
#define GREEN_LEDS_MOVE 4
#define RED_LEDS_MOVE 5
#define BLUE_LEDS_MOVE 6
#define GREEN_LEDS_RESET 7

#define WHEEL_DISTANCE 0.041		// Distance between wheels in meters (axis length).
#define ROBOT_RADIUS 0.025			// meters.

#define NUMBER_OF_DISTANCE_SENSORS 8
#define SENSOR_VALUE_DETECTION_THRESHOLD 50

#define COLLISION_VLT 5

int NOTHING = 0;
int FORWARD = 1;
int BACKWARD = 2;

bool trigger_motive = false;

// CHANGED
int robotAddress[2];
//

bool enabledSensors[SENSORS_NUM];
bool changedActuators_0[ACTUATORS_NUM];
bool changedActuators_1[ACTUATORS_NUM];
// CHANGED
// int speedLeft_0 = 0, speedRight_0 = 0, speedLeft_1 = 0, speedRight_1 = 0;
unsigned char ledNum = 0, ledState = 0;
//std::string elisa3Name;
std::string baseTag;
std::string nodeName;
//struct timeval currentTime2, lastTime2;
//struct timeval currentTime3, lastTime3;

//signed int accData[3];
//unsigned int floorData[4];
//unsigned int proxData[8];

//signed int robTheta=0, robXPos=0, robYPos=0;

ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;

//ros::Publisher odomPublisher;
//nav_msgs::Odometry odomMsg;

//std::map<int, ros::Publisher> odomPublishers;
//std::map<int, ros::Publisher> odomPublishers;
std::map<int, ros::Publisher> optiPublishers;

ros::Publisher accelPublisher;
sensor_msgs::Imu accelMsg;
ros::Publisher floorPublisher;
visualization_msgs::Marker floorMsg;

std::map<int, ros::Subscriber> VelSubscribers;

std::map<int, ros::Subscriber> GreenLedSubscribers;
std::map<int, ros::Subscriber> RedLedSubscribers;
std::map<int, ros::Subscriber> BlueLedSubscribers;
ros::Subscriber AllLedSubscriber;

std::map<int, ros::Subscriber> TargetSubscribers;
std::map<int, ros::Subscriber> ResetSubscribers;
ros::Subscriber AllResetSubscriber;
std::map<int, ros::Subscriber> OptiSubscribers;
std::map<int, ros::Subscriber> AutoMotiveSubscribers;
ros::Subscriber AllAutoMotiveSubscriber;

//double xPos, yPos, theta;
//double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
//double deltaXCorr, deltaYCorr;
//double xPosCorr, yPosCorr;
//double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;
//ros::Time currentTime, lastTime;

//ros::Time currentTimeMap, lastTimeMap;

//tf2_ros::Buffer tfBuffer;
//tf2_ros::TransformListener tfListener(tfBuffer);

class Robot{
	public:
	double speedLeft, speedRight;
	int greenLed =0, greenLed_move=0, greenLed_reset=0;
    int redLed =0, redLed_move=0;
    int blueLed =0, blueLed_move=0;
	int count, address;

	int auto_move_after_trigger = false;

//	std::string tag;
    int tag;
	bool changedActuators[ACTUATORS_NUM];

	int robTheta, robXPos, robYPos;

	double xPos, yPos, zPos, theta;
	double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
	double deltaXCorr, deltaYCorr;
	double xPosCorr, yPosCorr;
	double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;

	int trigger_delay = 0;

	double x_target, y_target;

//	ros::Time currentTime;
//    ros::Time lastTime = ros::Time::now();

	nav_msgs::Odometry odomMsg;
//    geometry_msgs::PoseStamped optiMsg;

    geometry_msgs::TransformStamped odomTrans;
    geometry_msgs::TransformStamped odomTransInit;

    // Obstacle Avoidance
    unsigned int proxData[8];
    int move_type = NOTHING;

    void updateSensorData();
	void updateRosInfo();

	bool reset = false;
};

void Robot::updateSensorData() {
    robXPos = getOdomXpos(address);
    robYPos = getOdomYpos(address);
    robTheta = getOdomTheta(address);
//    getAllProximity(address, proxData);
}

std::map<int, Robot> robots_dict;
//std::map<std::string, Robot> robots_dict;

void Robot::updateRosInfo() {
//    if (robots_dict[tag].reset) {
//        robDeltaX = 0;
//        robDeltaY = 0;
//        robDeltaTheta = 0;
//    } else {
//        robDeltaX = robXPos - robXPosPrev;
//        robDeltaY = robYPos - robYPosPrev;
//        robDeltaTheta = (robTheta - robThetaPrev) * M_PI / 180;
//    }
//    robXPosPrev = robXPos;
//    robYPosPrev = robYPos;
//    robThetaPrev = robTheta;
//
////    theta = robTheta*M_PI/180;    // Expressed in radiant.
//    if (robots_dict[tag].reset) {
//        theta = robTheta * M_PI / 180;
//    } else {
//        theta += robDeltaTheta;    // Expressed in radiant.
//    }
//
//    // We noticed from field tests on a vertical wall that there is a difference in the measured distance between
//    // a route traveled toward bottom and a route traveled toward top. For this reason we adjust the distance
//    // traveled based on the angle.
//    //robots_dict[it->first].robDeltaDistTraveled = sqrt(robots_dict[it->first].robDeltaX*robots_dict[it->first].robDeltaX + robots_dict[it->first].robDeltaY*robots_dict[it->first].robDeltaY);
//
//    if(robTheta <= 180 && robTheta >= 0) {
//        robDistTraveled = sqrt(robDeltaX*robDeltaX + robDeltaY*robDeltaY);
//        deltaXCorr = robDistTraveled*2/3*cos(theta);
//        // 2/3 is the magical factor found from filed tests, probably you'll need to adapt it to your surface.
//        deltaYCorr = robDistTraveled*2/3*sin(theta);
//
//        //deltaXCorr = robDeltaDistTraveled*2/3*cos(theta);
//        deltaYCorr = robDeltaDistTraveled*2/3*sin(theta);
//
//    } else {
//        deltaXCorr = robDeltaX;
//        deltaYCorr = robDeltaY;
//    }
//
//    if (robots_dict[tag].reset){
//        xPos = robXPos/1000.0;
//        yPos = robYPos/1000.0;
//    } else {
//        xPos += deltaXCorr / 1000.0;    // Expressed in meters.
//        yPos += deltaYCorr / 1000.0;    // Expressed in meters.
//    }
//

    xPos = robXPos/ 1000.0;
    yPos = robYPos/ 1000.0;
    theta = robTheta * M_PI / 180;
    
//    xPos = robXPos/1000.0;   // Expressed in meters.
//    yPos = robYPos/1000.0;   // Expressed in meters.

//    robDeltaDistTraveled = (robDistTraveled - robDistTraveledPrev)/1000.0;
//    robDistTraveledPrev = robDistTraveled;

    // broadcast odom over tf
    odomTrans.header.stamp = ros::Time::now();
    odomTrans.transform.translation.x = xPos;
    odomTrans.transform.translation.y = yPos;
    odomTrans.transform.translation.z = 0.0;

    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(theta);
    odomTrans.transform.rotation = odomQuat_trans;

    // broadcast odomInit over tf
    odomTransInit.header.stamp = ros::Time::now();

//    // Publish the odometry message over ROS.
//    odomMsg.header.stamp = ros::Time::now();
//
//    odomMsg.pose.pose.position.x = xPos;
//    odomMsg.pose.pose.position.y = yPos;
//    odomMsg.pose.pose.position.z = theta;
//
//    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
//    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
//    odomMsg.pose.pose.orientation = odomQuat;
//
//    currentTime = ros::Time::now();
//    odomMsg.twist.twist.linear.x = robDeltaDistTraveled / ((currentTime-lastTime).toSec());
//    // "robDeltaDistTraveled" is the linear distance covered in meters from the last update (delta distance);
//    // the time from the last update is measured in seconds thus to get m/s we multiply them.
//    odomMsg.twist.twist.angular.z = robDeltaTheta / ((currentTime-lastTime).toSec());
//    // "robDeltaTheta" is the angular distance covered in radiant from the last update (delta angle);
//    // the time from the last update is measured in seconds thus to get rad/s we multiply them.
//    lastTime = ros::Time::now();

}

void updateSensorsData() {
	std::map<int, Robot>::iterator it;

	for (it = robots_dict.begin(); it != robots_dict.end(); it++){
	    robots_dict[it->first].updateSensorData();
	}
}

void updateRosInfo() {
	static tf2_ros::TransformBroadcaster br;

    std::map<int, Robot>::iterator it;
	for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        robots_dict[it->first].updateRosInfo();

//        optiPublishers[it->first].publish(robots_dict[it->first].optiMsg);

        br.sendTransform(robots_dict[it->first].odomTrans);
        br.sendTransform(robots_dict[it->first].odomTransInit);
	}
}
//
//void handlerOpti(const geometry_msgs::PoseStamped::ConstPtr& msg, int tag) {
//    tf::Quaternion q(
//            msg->pose.orientation.x,
//            msg->pose.orientation.y,
//            msg->pose.orientation.z,
//            msg->pose.orientation.w);
//    tf::Matrix3x3 m(q);
//    double roll, pitch, yaw;
//    m.getRPY(roll, pitch, yaw);
//
//    if (roll < 0.75 && pitch < 0.75){
//        robots_dict[tag].optiMsg.pose.position.x = double(msg->pose.position.x);
//        robots_dict[tag].optiMsg.pose.position.y = double(msg->pose.position.y);
//        robots_dict[tag].optiMsg.pose.position.z = yaw;
//        robots_dict[tag].optiMsg.pose.orientation.x = double(msg->pose.orientation.x);
//        robots_dict[tag].optiMsg.pose.orientation.y = double(msg->pose.orientation.y);
//        robots_dict[tag].optiMsg.pose.orientation.z = double(msg->pose.orientation.z);
//        robots_dict[tag].optiMsg.pose.orientation.w = double(msg->pose.orientation.w);
//        robots_dict[tag].optiMsg.header.stamp = ros::Time::now();
//    }
//}

int getIdFromAddress(int address) {
    std::map<int, Robot>::iterator it;
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        if(robots_dict[it->first].address == address) {
            return int(it->first);
        }
    }
    return -1;
}

// OLD FUNCTION, HANDLING PER ROBOT
//void handlerReset(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
//    double type;
//    type = double(msg->data[0]);
//
//    robots_dict[tag].odomTransInit.header.stamp = ros::Time::now();
//    if (type == 0){
//        robots_dict[tag].odomTransInit.transform.translation.x = double(msg->data[1]);
//        robots_dict[tag].odomTransInit.transform.translation.y = double(msg->data[2]);
//        robots_dict[tag].odomTransInit.transform.translation.z = 0.0;
//        geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(double(msg->data[3]));
//        robots_dict[tag].odomTransInit.transform.rotation = odomQuat_trans;
//
//        robots_dict[tag].changedActuators[GREEN_LEDS] = true;
//        if (robots_dict[tag].greenLed ==101) {
//            robots_dict[tag].greenLed = 102;
//        } else {
//            robots_dict[tag].greenLed = 101;
//        }
//        robots_dict[tag].reset = true;
//
//    } else if (type == 1) {
//        robots_dict[tag].odomTransInit.transform.translation.x =robots_dict[tag].optiMsg.pose.position.x;
//        robots_dict[tag].odomTransInit.transform.translation.y =robots_dict[tag].optiMsg.pose.position.y;
//        robots_dict[tag].odomTransInit.transform.translation.z =robots_dict[tag].optiMsg.pose.position.z;
//        robots_dict[tag].odomTransInit.transform.rotation.x = robots_dict[tag].optiMsg.pose.orientation.x;
//        robots_dict[tag].odomTransInit.transform.rotation.y = robots_dict[tag].optiMsg.pose.orientation.y;
//        robots_dict[tag].odomTransInit.transform.rotation.z = robots_dict[tag].optiMsg.pose.orientation.z;
//        robots_dict[tag].odomTransInit.transform.rotation.w = robots_dict[tag].optiMsg.pose.orientation.w;
//
//        robots_dict[tag].changedActuators[GREEN_LEDS] = true;
//        if (robots_dict[tag].greenLed ==101) {
//            robots_dict[tag].greenLed = 102;
//        } else {
//            robots_dict[tag].greenLed = 101;
//        }
//        robots_dict[tag].reset = true;
//    }
//}

void handlerAllReset(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int nr_robots = int(msg->data[0]);
//    std::cout << "[" << nodeName << "] " << "nr robots resetting: " << nr_robots << std::endl;
    int tag;

    XmlRpc::XmlRpcValue::iterator i;
    for (int i=0; i < nr_robots; i++){
        tag = getIdFromAddress(int(msg->data[i*5+1]));

        double type;
        type = double(msg->data[i*5+2]);

        robots_dict[tag].odomTransInit.header.stamp = ros::Time::now();
        if (type == 0){
            robots_dict[tag].odomTransInit.transform.translation.x = double(msg->data[i*5+3]);
            robots_dict[tag].odomTransInit.transform.translation.y = double(msg->data[i*5+4]);
            robots_dict[tag].odomTransInit.transform.translation.z = 0.0;
            geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(double(msg->data[i*5+5]));
            robots_dict[tag].odomTransInit.transform.rotation = odomQuat_trans;

            robots_dict[tag].changedActuators[GREEN_LEDS_RESET] = true;
            if (robots_dict[tag].greenLed_reset ==101) {
                robots_dict[tag].greenLed_reset = 102;
            } else {
                robots_dict[tag].greenLed_reset = 101;
            }
            robots_dict[tag].reset = true;
        }

        
//        else if (type == 1) {
//            robots_dict[tag].odomTransInit.transform.translation.x =robots_dict[tag].optiMsg.pose.position.x;
//            robots_dict[tag].odomTransInit.transform.translation.y =robots_dict[tag].optiMsg.pose.position.y;
//            robots_dict[tag].odomTransInit.transform.translation.z =robots_dict[tag].optiMsg.pose.position.z;
//            robots_dict[tag].odomTransInit.transform.rotation.x = robots_dict[tag].optiMsg.pose.orientation.x;
//            robots_dict[tag].odomTransInit.transform.rotation.y = robots_dict[tag].optiMsg.pose.orientation.y;
//            robots_dict[tag].odomTransInit.transform.rotation.z = robots_dict[tag].optiMsg.pose.orientation.z;
//            robots_dict[tag].odomTransInit.transform.rotation.w = robots_dict[tag].optiMsg.pose.orientation.w;
//
//            robots_dict[tag].changedActuators[GREEN_LEDS_RESET] = true;
//            if (robots_dict[tag].greenLed_reset ==101) {
//                robots_dict[tag].greenLed_reset = 102;
//            } else {
//                robots_dict[tag].greenLed_reset = 101;
//            }
//            robots_dict[tag].reset = true;
//        }
    }
}

// OLD FUNCTION, HANDLING PER ROBOT
//void handlerAutoMove(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
//    int turn_type = int(msg->data[0]);
//    int trans_type = int(msg->data[2]);
//    if (turn_type==0){
//        int prop_msg = int(double(double(msg->data[1])*(75.0/(2.0*M_PI))+100.0));
//        if (prop_msg >= 175) {
//            prop_msg = 175;
//        }
//        robots_dict[tag].redLed_move = prop_msg;
//    } else if (turn_type==1){
//        int prop_msg = int(double(double(msg->data[1])*(75.0/(2.0*M_PI))+175.0));
//        if (prop_msg >= 250) {
//            prop_msg = 250;
//        } else if (prop_msg == 175){
//            prop_msg = 100;
//        }
//        robots_dict[tag].redLed_move = prop_msg;
//    } else {
//        robots_dict[tag].redLed_move = 100;
//    }
//
//    if (trans_type==0){
//        int prop_msg = int(double(msg->data[3])*25 + 100);
//        if (prop_msg >= 175){
//            prop_msg = 175;
//        }
//        robots_dict[tag].blueLed_move = prop_msg;
//    } else {
//        int prop_msg = int(double(msg->data[3])*25 + 175);
//        if (prop_msg >= 250){
//            prop_msg = 250;
//        } else if (prop_msg = 175){
//            prop_msg = 100;
//        }
//        robots_dict[tag].blueLed_move = prop_msg;
//    }
//
//    if (robots_dict[tag].greenLed_move ==103) {
//        robots_dict[tag].greenLed_move = 104;
//    } else {
//        robots_dict[tag].greenLed_move = 103;
//    }
//    robots_dict[tag].changedActuators[GREEN_LEDS_MOVE] = true;
//    robots_dict[tag].changedActuators[RED_LEDS_MOVE] = true;
//    robots_dict[tag].changedActuators[BLUE_LEDS_MOVE] = true;
//    std::cout << "[" << nodeName << "] " << "[robot " << tag <<
//            "]" << "handler green led: " << robots_dict[tag].greenLed_move  << std::endl;
//    std::cout << "[" << nodeName << "] " << "[robot " << tag <<
//        "]" << "handler red led: " << robots_dict[tag].redLed_move  << std::endl;
//    std::cout << "[" << nodeName << "] " << "[robot " << tag <<
//        "]" << "handler blue led: " << robots_dict[tag].blueLed_move  << std::endl;
//}

void handlerAllAutoMove(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int nr_robots = int(msg->data[0]);
//    std::cout << "[" << nodeName << "] " << "nr robots: " << nr_robots << std::endl;
    int tag;

    XmlRpc::XmlRpcValue::iterator i;
    for (int i=0; i < nr_robots; i++){
        tag = getIdFromAddress(int(msg->data[i*5+1]));

        // PER ROBOT HANDLING
        int turn_type = int(msg->data[i*5+2]);
        int trans_type = int(msg->data[i*5+4]);
        if (turn_type==0){
            int prop_msg = int(double(double(msg->data[i*5+3])*(75.0/(2.0*M_PI))+100.0));
            if (prop_msg >= 175) {
                prop_msg = 175;
            }
            robots_dict[tag].redLed_move = prop_msg;
        } else if (turn_type==1){
            int prop_msg = int(double(double(msg->data[i*5+3])*(75.0/(2.0*M_PI))+175.0));
            if (prop_msg >= 250) {
                prop_msg = 250;
            } else if (prop_msg == 175){
                prop_msg = 100;
            }
            robots_dict[tag].redLed_move = prop_msg;
        } else {
            robots_dict[tag].redLed_move = 100;
        }

        if (trans_type==0){
            int prop_msg = int(double(msg->data[i*5+5])*25 + 100);
            if (prop_msg >= 175){
                prop_msg = 175;
            }
            robots_dict[tag].blueLed_move = prop_msg;
        } else {
            int prop_msg = int(double(msg->data[i*5+5])*25 + 175);
            if (prop_msg >= 250){
                prop_msg = 250;
            } else if (prop_msg = 175){
                prop_msg = 100;
            }
            robots_dict[tag].blueLed_move = prop_msg;
        }

        if (robots_dict[tag].greenLed_move ==103) {
            robots_dict[tag].greenLed_move = 104;
        } else {
            robots_dict[tag].greenLed_move = 103;
        }
        robots_dict[tag].changedActuators[GREEN_LEDS_MOVE] = true;
        robots_dict[tag].changedActuators[RED_LEDS_MOVE] = true;
        robots_dict[tag].changedActuators[BLUE_LEDS_MOVE] = true;
//
//
////        std::cout << "[" << nodeName << "] " << "[robot " << tag <<
////        "]" << "handler green led move: " << robots_dict[tag].greenLed_move  << std::endl;
////        std::cout << "[" << nodeName << "] " << "[robot " << tag <<
////        "]" << "handler red led move: " << robots_dict[tag].redLed_move  << std::endl;
////        std::cout << "[" << nodeName << "] " << "[robot " << tag <<
////        "]" << "handler blue led move: " << robots_dict[tag].blueLed_move  << std::endl;
    }
}

//void handlerVelocity(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
//    // Remove move type
//	double	speedLeft;
//	double speedRight;
//	speedLeft = double(msg->data[0]);
//	speedRight = double(msg->data[1]);
//    robots_dict[tag].move_type = int(msg->data[2]);  //0: Not Move, 1: Move Forward, 2:Move Backward
//
//	if(speedLeft>127) {
//		speedLeft = 127;
//	}
//	if(speedRight>127){
//		speedRight=127;
//	}
//	if(speedLeft<-127){
//		speedLeft=-127;
//	}
//	if(speedRight<-127){
//		speedRight=-127;
//	}
//
//	robots_dict[tag].changedActuators[MOTORS] = true;
//	robots_dict[tag].speedLeft = speedLeft;
//	robots_dict[tag].speedRight = speedRight;
//
//	std::cout << "[" << nodeName << "] " << "[" << baseTag + "_" << tag << "] " << "speedLeft: " << speedLeft << std::endl;
//	std::cout << "[" << nodeName << "] " << "[" << baseTag + "_" << tag << "] " << "speedRight: " << speedRight << std::endl;
//    std::cout << "[" << nodeName << "] " << "[" << baseTag + "_" << tag << "] " << "move type " << robots_dict[tag].move_type << std::endl;
//}

void handlerAllLeds(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int nr_robots;
    nr_robots = int(msg->data[0]);
    int tag;

    XmlRpc::XmlRpcValue::iterator i;
    for (int i=0; i < nr_robots; i++){
        tag = getIdFromAddress(int(msg->data[i*4+1]));
        robots_dict[tag].changedActuators[GREEN_LEDS] = true;
        robots_dict[tag].changedActuators[RED_LEDS] = true;
        robots_dict[tag].changedActuators[BLUE_LEDS] = true;
        robots_dict[tag].greenLed = int(msg->data[i*4+2]);
        robots_dict[tag].redLed = int(msg->data[i*4+3]);
        robots_dict[tag].blueLed = int(msg->data[i*4+4]);

//        std::cout << "[" << nodeName << "] " << "[robot " << tag <<
//        "]" << "handler green led: " << robots_dict[tag].greenLed  << std::endl;
//        std::cout << "[" << nodeName << "] " << "[robot " << tag <<
//        "]" << "handler red led: " << robots_dict[tag].redLed  << std::endl;
//        std::cout << "[" << nodeName << "] " << "[robot " << tag <<
//        "]" << "handler blue led: " << robots_dict[tag].blueLed  << std::endl;
    }


}

//void handlerGreenLed(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
//	int	intensity;
//	intensity = int(msg->data[0]);
//	robots_dict[tag].changedActuators[GREEN_LEDS] = true;
//	robots_dict[tag].greenLed = intensity;
//}

//void handlerRedLed(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
//    int	intensity;
//    intensity = int(msg->data[0]);
//    robots_dict[tag].changedActuators[RED_LEDS] = true;
//    robots_dict[tag].redLed = intensity;
//}

//void handlerBlueLed(const std_msgs::Float64MultiArray::ConstPtr& msg, int tag) {
//    int	intensity;
//    intensity = int(msg->data[0]);
//    robots_dict[tag].changedActuators[BLUE_LEDS] = true;
//    robots_dict[tag].blueLed = intensity;
//}

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
    
    char buff[6];

	std::map<int, Robot>::iterator it;

	for (it = robots_dict.begin(); it != robots_dict.end(); it++){
	    bool *is_sensor_active = get_sensors_condition(robots_dict[it->first].proxData);

        if(robots_dict[it->first].changedActuators[MOTORS]){
            robots_dict[it->first].changedActuators[MOTORS] = false;

            //std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].count << "]" <<
            // "updated leftSpeed: " << robots_dict[it->first].speedLeft << std::endl;
            //std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].count << "]" <<
            // "updated rightSpeed: " << robots_dict[it->first].speedRight << std::endl;

            setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft);
            setRightSpeed(robots_dict[it->first].address, robots_dict[it->first].speedRight);
		}

        if(robots_dict[it->first].changedActuators[BLUE_LEDS] and
        robots_dict[it->first].changedActuators[RED_LEDS] and
        robots_dict[it->first].changedActuators[GREEN_LEDS]) {
            robots_dict[it->first].changedActuators[RED_LEDS] = false;
            robots_dict[it->first].changedActuators[BLUE_LEDS] = false;
            robots_dict[it->first].changedActuators[GREEN_LEDS] = false;
            //            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
            //            "]" << "updated blue leds move: " << robots_dict[it->first].blueLed_move << std::endl;
            setAllColors(robots_dict[it->first].address, robots_dict[it->first].redLed,
                         robots_dict[it->first].greenLed, robots_dict[it->first].blueLed);
        }

//        // CHECK RED LED
//        if(robots_dict[it->first].changedActuators[RED_LEDS]){
//            robots_dict[it->first].changedActuators[RED_LEDS] = false;
////            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
////                      "]" << "updated red leds: " << robots_dict[it->first].redLed << std::endl;
//            setRed(robots_dict[it->first].address, robots_dict[it->first].redLed);
//        }
//        // CHECK BLUE LED
//        if(robots_dict[it->first].changedActuators[BLUE_LEDS]) {
//            robots_dict[it->first].changedActuators[BLUE_LEDS] = false;
////            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
////                      "]" << "updated blue leds: " << robots_dict[it->first].blueLed << std::endl;
//            setBlue(robots_dict[it->first].address, robots_dict[it->first].blueLed);
//        }
//
//        // CHECK GREEN LED
//        if(robots_dict[it->first].changedActuators[GREEN_LEDS]){
//            robots_dict[it->first].changedActuators[GREEN_LEDS] = false;
//            //			std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
//            //			            "]" << "updated green leds: " << robots_dict[it->first].greenLed << std::endl;
//            setGreen(robots_dict[it->first].address, robots_dict[it->first].greenLed);
//        }

//        // CHECK RED LED MOVE
//        if(robots_dict[it->first].changedActuators[RED_LEDS_MOVE]){
//            robots_dict[it->first].changedActuators[RED_LEDS_MOVE] = false;
////            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
////            "]" << "updated red leds move: " << robots_dict[it->first].redLed_move << std::endl;
//            setRed(robots_dict[it->first].address, robots_dict[it->first].redLed_move);
//        }
//        // CHECK BLUE LED MOVE
//        if(robots_dict[it->first].changedActuators[BLUE_LEDS_MOVE]) {
//            robots_dict[it->first].changedActuators[BLUE_LEDS_MOVE] = false;
////            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
////            "]" << "updated blue leds move: " << robots_dict[it->first].blueLed_move << std::endl;
//            setBlue(robots_dict[it->first].address, robots_dict[it->first].blueLed_move);
//        }
//        // CHECK GREEN LED MOVE
//        if(robots_dict[it->first].changedActuators[GREEN_LEDS_MOVE]){
//            robots_dict[it->first].changedActuators[GREEN_LEDS_MOVE] = false;
//            //            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
//            //            "]" << "updated green leds move: " << robots_dict[it->first].greenLed_move << std::endl;
//            setGreen(robots_dict[it->first].address, robots_dict[it->first].greenLed_move);
//            robots_dict[it->first].greenLed_move_counter -=1;
//        }

        if(robots_dict[it->first].changedActuators[BLUE_LEDS_MOVE] and
            robots_dict[it->first].changedActuators[RED_LEDS_MOVE] and
            robots_dict[it->first].changedActuators[GREEN_LEDS_MOVE]) {
            robots_dict[it->first].changedActuators[RED_LEDS_MOVE] = false;
            robots_dict[it->first].changedActuators[BLUE_LEDS_MOVE] = false;
            robots_dict[it->first].changedActuators[GREEN_LEDS_MOVE] = false;
            //            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
            //            "]" << "updated blue leds move: " << robots_dict[it->first].blueLed_move << std::endl;
            setAllColors(robots_dict[it->first].address, robots_dict[it->first].redLed_move,
                         robots_dict[it->first].greenLed_move, robots_dict[it->first].blueLed_move);

            trigger_motive = true;

        }

        // CHECK GREEN LED RESET
        if(robots_dict[it->first].changedActuators[GREEN_LEDS_RESET]){
            robots_dict[it->first].changedActuators[GREEN_LEDS_RESET] = false;
//            std::cout << "[" << nodeName << "] " << "[robot " << robots_dict[it->first].tag <<
//            "]" << "updated green leds reset: " << robots_dict[it->first].greenLed_reset << std::endl;
            setGreen(robots_dict[it->first].address, robots_dict[it->first].greenLed_reset);
        }
	}

//	if (trigger_motive){
//	    trigger_motive = false;
//	    ros::Duration(2).sleep();
//	    std::map<int, Robot>::iterator it2;
//	    for (it2 = robots_dict.begin(); it2 != robots_dict.end(); it2++){
//	        setAllColors(robots_dict[it2->first].address, robots_dict[it2->first].redLed,
//                         robots_dict[it2->first].greenLed_move, robots_dict[it2->first].blueLed);
//	    }
//	}

}


int main(int argc,char *argv[]) {
//		double init_xpos, init_ypos, init_theta;
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

//	np.param<std::string>("base_tag", elisa3Name, "elisa3");

	np.param<std::string>("base_tag", baseTag, "elisa3");
	np.param<std::string>("name", nodeName, "elisa3");

	XmlRpc::XmlRpcValue body_list;
    np.param("rigid_bodies", body_list,body_list);

	int N_robots = body_list.size();
	int robot_addresses[N_robots];
//	std::cout << "[" << nodeName << "] " << "number of robots: " << N_robots << std::endl;

	//cmdGreenLed = n.subscribe("elisa3_robot/green_led",10,handlerGreenLed);
	//cmdVelSubscriber = n.subscribe("elisa3_robot/mobile_base/cmd_vel",10,handlerVelocity);

    if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0){
		XmlRpc::XmlRpcValue::iterator i;
		
		int count = 0;
		for (i = body_list.begin(); i != body_list.end(); ++i) {
//		    std::cout << "[" << nodeName << "] " << "init number:: " << count << std::endl;
			Robot class_inst;
			class_inst.speedLeft =0;
			class_inst.speedRight = 0;
			class_inst.greenLed=0;

            class_inst.count = count;
            class_inst.address = (i->second)["address"];
//            class_inst.currentTime = ros::Time::now();

            class_inst.xPos = getOdomXpos(class_inst.address);
            class_inst.yPos = getOdomYpos(class_inst.address);
            class_inst.theta = getOdomTheta(class_inst.address);

			class_inst.changedActuators[MOTORS] = false;
            class_inst.tag = atoi((i->first).c_str());

            class_inst.odomTrans.header.frame_id  = "elisa3_robot_"+ std::to_string(class_inst.tag) + "_init";
            class_inst.odomTrans.child_frame_id = "elisa3_robot_"+ std::to_string(class_inst.tag);

            class_inst.odomTransInit.header.frame_id  = "world";
            class_inst.odomTransInit.child_frame_id = "elisa3_robot_"+ std::to_string(class_inst.tag) + "_init";
            class_inst.odomTransInit.header.stamp = ros::Time::now();
            class_inst.odomTransInit.transform.translation.x = class_inst.xPos;
            class_inst.odomTransInit.transform.translation.y = class_inst.yPos;
            class_inst.odomTransInit.transform.translation.z = 0.0;
            geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(class_inst.theta);
            class_inst.odomTransInit.transform.rotation = odomQuat_trans;

            robots_dict[class_inst.tag] = class_inst;

            robotAddress[count] = class_inst.address;
            robot_addresses[count] = class_inst.address;

//			boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> f = boost::bind(handlerVelocity, _1, class_inst.tag);
//            VelSubscribers[class_inst.tag] = n.subscribe("elisa3_robot_"+ std::to_string(class_inst.tag) +"/mobile_base/cmd_vel", 1000, f);

//			boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> g = boost::bind(handlerGreenLed, _1, class_inst.tag);
//            GreenLedSubscribers[class_inst.tag] = n.subscribe("elisa3_robot_"+ std::to_string(class_inst.tag) +"/green_led", 1000, g);

//            boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> k = boost::bind(handlerRedLed, _1, class_inst.tag);
//            RedLedSubscribers[class_inst.tag] = n.subscribe("elisa3_robot_"+ std::to_string(class_inst.tag) +"/red_led", 1000, k);

//            boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> l = boost::bind(handlerBlueLed, _1, class_inst.tag);
//            BlueLedSubscribers[class_inst.tag] = n.subscribe("elisa3_robot_"+ std::to_string(class_inst.tag) +"/blue_led", 1000, l);

            AllLedSubscriber = n.subscribe("elisa3_all_robots/leds", 10, handlerAllLeds);

//            boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> i = boost::bind(handlerReset, _1, class_inst.tag);
//            ResetSubscribers[class_inst.tag] = n.subscribe("elisa3_robot_"+ std::to_string(class_inst.tag) +"/reset", 1000, i);

            AllResetSubscriber = n.subscribe("elisa3_all_robots/reset", 10, handlerAllReset);

//            boost::function<void (const geometry_msgs::PoseStamped::ConstPtr&)> j = boost::bind(handlerOpti, _1, class_inst.tag);
//            OptiSubscribers[class_inst.tag] = n.subscribe("rigid_body_"+ std::to_string(class_inst.tag) +"/pose", 1000, j);

//            boost::function<void (const std_msgs::Float64MultiArray::ConstPtr&)> m = boost::bind(handlerAutoMove, _1, class_inst.tag);
//            AutoMotiveSubscribers[class_inst.tag] = n.subscribe("elisa3_robot_"+ std::to_string(class_inst.tag) +"/auto_motive", 1000, m);

            AllAutoMotiveSubscriber = n.subscribe("elisa3_all_robots/auto_motive", 10, handlerAllAutoMove);

//            odomPublishers[class_inst.tag] = n.advertise<nav_msgs::Odometry>(
//                    "elisa3_robot_"+ std::to_string(class_inst.tag) +"/odom", 10);
//            optiPublishers[class_inst.tag] = n.advertise<geometry_msgs::PoseStamped>(
//                    "elisa3_robot_"+ std::to_string(class_inst.tag) +"/opti", 1000);

            // LOG
//			std::cout << "[" << nodeName << "] " << "[robot_" << class_inst.tag << "] " << "address: " << class_inst.address << "count: " << count << std::endl;

			count += 1;


		}
	}

//    np.param("xpos", init_xpos, 0.0);
//    np.param("ypos", init_ypos, 0.0);
//    np.param("theta", init_theta, 0.0);
    np.param("accelerometer", enabledSensors[ACCELEROMETER], false);
    np.param("floor", enabledSensors[FLOOR], false);
    np.param("proximity", enabledSensors[PROXIMITY], false);
    np.param("motor_position", enabledSensors[MOTOR_POSITION], false);


    //if(DEBUG_ROS_PARAMS) {
    //    std::cout << "[" << nodeName << "] " << "init pose: " << init_xpos << ", " << init_ypos << ", " << theta << std::endl;
     //   std::cout << "[" << nodeName << "] " << "accelerometer enabled: " << enabledSensors[ACCELEROMETER] << std::endl;
     //   std::cout << "[" << nodeName << "] " << "floor enabled: " << enabledSensors[FLOOR] << std::endl;
     //   std::cout << "[" << nodeName << "] " << "proximity enabled: " << enabledSensors[PROXIMITY] << std::endl;
     //   std::cout << "[" << nodeName << "] " << "motor position enabled: " << enabledSensors[MOTOR_POSITION] << std::endl;
    //}

    //startCommunication(robotAddress, 2);
    startCommunication(robot_addresses, N_robots);
    

    //if(enabledSensors[ACCELEROMETER]) {
	//			sensorsEnabled++;
     //   accelPublisher = n.advertise<sensor_msgs::Imu>("accel", 10);
    //}
    //if(enabledSensors[FLOOR]) {
		//		sensorsEnabled++;
        //floorPublisher = n.advertise<visualization_msgs::Marker>("floor", 10);
    //}
    //if(enabledSensors[PROXIMITY]) {
		//		sensorsEnabled++;
        //for(i=0; i<8; i++) {
            /**
            * The advertise() function is how you tell ROS that you want to
            * publish on a given topic name. This invokes a call to the ROS
            * master node, which keeps a registry of who is publishing and who
            * is subscribing. After this advertise() call is made, the master
            * node will notify anyone who is trying to subscribe to this topic name,
            * and they will in turn negotiate a peer-to-peer connection with this
            * node.  advertise() returns a Publisher object which allows you to
            * publish messages on that topic through a call to publish().  Once
            * all copies of the returned Publisher object are destroyed, the topic
            * will be automatically unadvertised.
            *
            * The second parameter to advertise() is the size of the message queue
            * used for publishing messages.  If messages are published more quickly
            * than we can send them, the number here specifies how many messages to
            * buffer up before throwing some away.
            */
           // std::stringstream ss;
           // ss.str("");
           // ss << "proximity" << i;
           // proxPublisher[i] = n.advertise<sensor_msgs::Range>(ss.str(), 10);
           // //proxMsg[i] = new sensor_msgs::Range();
           // proxMsg[i].radiation_type = sensor_msgs::Range::INFRARED;
           // ss.str("");
           // ss << elisa3Name << "/base_prox" << i;
           // proxMsg[i].header.frame_id =  ss.str();
           // proxMsg[i].field_of_view = 0.26;    // About 15 degrees...to be checked!
          //  proxMsg[i].min_range = 0.005;       // 0.5 cm.
          //  proxMsg[i].max_range = 0.05;        // 5 cm.                    
      // }       
        
       // laserPublisher = n.advertise<sensor_msgs::LaserScan>("scan", 10);
   // }
    //if(enabledSensors[MOTOR_POSITION]) {
	//			sensorsEnabled++;
       // odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 10);
        //currentTime = ros::Time::now();
        //lastTime = ros::Time::now();        
    //}

   // if(sensorsEnabled == 0) {
    //    std::cerr << "[" << elisa3Name << "] " << "No sensors enabled!" << std::endl;
	//			stopCommunication();
    //    return -1;
   // }
       
    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called handlerVelocity.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */

    //cmdVelSubscriber_0 = n.subscribe("elisa3_robot_1/mobile_base/cmd_vel", 10, handlerVelocity_0);    
    //cmdVelSubscriber_1 = n.subscribe("elisa3_robot_2/mobile_base/cmd_vel", 10, handlerVelocity_1); 

	//cmdGreenLed_0 = n.subscribe("elisa3_robot_1/green_led",10,handlerGreenLed_0);
	//cmdGreenLed_1 = n.subscribe("elisa3_robot_2/green_led",10,handlerGreenLed_1);

	

    //theta = init_theta;
    //xPos = init_xpos;
    //yPos = init_ypos;
    std::map<int, Robot>::iterator it;
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        enableObstacleAvoidance(robots_dict[it->first].address);
    }
		// CHANGED
		//enableObstacleAvoidance(robotAddress[0]);

		//enableTVRemote(robotAddress[1]);

    while (ros::ok()) {
        updateSensorsData();
        updateRosInfo();
        updateActuators();
        ros::spinOnce();
				//if(waitForUpdate(robotAddress[0], 10000000)) { // Wait for at most 10 seconds.
					//	break; // We have connection problems, stop here.
				//}
    }

		stopCommunication();
}





