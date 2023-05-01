#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
// #include "elisa3-lib.h"
#include <ros/ros.h>
#include "pc-side-elisa3-library/elisa3-lib.h"
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


// for the camera
#include <geometry_msgs/Pose2D.h>


// #include "main_part.h"

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

bool trigger_motive = false;

bool enabledSensors[SENSORS_NUM];
bool changedActuators_0[ACTUATORS_NUM];
bool changedActuators_1[ACTUATORS_NUM];
double speedLeft = 0;
double speedRight = 0;
unsigned char ledNum = 0, ledState = 0;
std::string baseTag;
std::string nodeName;

ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;

ros::Publisher odomPublisher[100];

ros::Publisher accelPublisher[100];

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

// std::map<int, ros::Subscriber> OptiSubscribers;
std::map<int, ros::Subscriber> AutoMotiveSubscribers;
ros::Subscriber AllAutoMotiveSubscriber;





ros::Subscriber AllOptiSubscriber;

class Camera{
    public:
    double cam_x = 0.0, cam_y = 0.0, cam_phi = 0.0;
    int tag = 0;
};

std::map<int, Camera> cameras_dict;

// void Camera::updateSensorData(){
    
// }

// void Camera::updateRosInfo(){

// }





class Robot{
	public:
	double speedLeft, speedRight;
	int greenLed =0, greenLed_move=0, greenLed_reset=0;
    int redLed =0, redLed_move=0;
    int blueLed =0, blueLed_move=0;
	int count, address;

	int auto_move_after_trigger = false;

    int tag;
    int timer = 0;

	bool changedActuators[ACTUATORS_NUM];

	int robTheta, robXPos, robYPos;

	double xPos, yPos, zPos, theta;
    double xPosc, yPosc, zPosc;
	double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
	double deltaXCorr, deltaYCorr;
	double xPosCorr, yPosCorr;
	double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;

    double accx_raw_measure = 0, accx_raw_measure_prev = 0, accx_filtered_prev = 0, accx_filtered = 0;

	int trigger_delay = 0;
    signed int accData[3];
	double x_target, y_target;

	nav_msgs::Odometry odomMsg;
    sensor_msgs::Imu accelMsg;

    double time_prev = 0.0;

    geometry_msgs::TransformStamped odomTrans;
    geometry_msgs::TransformStamped odomTransInit;

    // Obstacle Avoidance
    unsigned int proxData[8];

    void updateSensorData();
	void updateRosInfo();


    void IIR_filter();

	bool reset = false;
};

std::map<int, Robot> robots_dict;


void Robot::updateSensorData() {
    robXPos = getOdomXpos(address);
    robYPos = getOdomYpos(address);
    robTheta = getOdomTheta(address);

// could add a filter there

    accData[0] = getAccX(address);
    accData[1] = getAccY(address);
    accData[2] = getAccZ(address);

    accx_raw_measure = accData[0];
    
    // std::cout << "[" << nodeName << "] " << "accData[0]: " << accData[0] << std::endl;
//    getAllProximity(address, proxData);
}

void Robot::updateRosInfo() {
    // Expressed in meters.
    xPos = robXPos/ 1000.0;
    yPos = robYPos/ 1000.0;
    // Expressed in radiant.
    theta = robTheta * M_PI / 180;
    

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

    
    // broadcast odom over tf
    odomTrans.header.stamp = ros::Time::now();
    odomTrans.transform.translation.x = xPos;
    odomTrans.transform.translation.y = yPos;
    odomTrans.transform.translation.z = 0.0;

    odomMsg.twist.twist.linear.x = xPosc;
    odomMsg.twist.twist.linear.y = yPosc;


    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    geometry_msgs::Quaternion odomQuat_trans = tf::createQuaternionMsgFromYaw(theta);
    odomTrans.transform.rotation = odomQuat_trans;

    // broadcast odomInit over tf
    odomTransInit.header.stamp = ros::Time::now();


    // accel info

    // low-pass filter first
    IIR_filter();

    std::stringstream ss3;
    ss3 << "elisa3_robot_" << "/base_link";
    accelMsg.header.frame_id = ss3.str();
    accelMsg.header.stamp = ros::Time::now();   

    
    // time_prev = ros::Time::now();     
    accelMsg.linear_acceleration.x = (accx_filtered)/64.0*9.81; // 1 g = 64, then transforms in m/s^2.
    accelMsg.linear_acceleration.y = (accData[1])/64.0*9.81;
    accelMsg.linear_acceleration.z = (accData[2])/64.0*9.81;
    accelMsg.linear_acceleration_covariance[0] = 0.01;
    accelMsg.linear_acceleration_covariance[1] = 0.0;
    accelMsg.linear_acceleration_covariance[2] = 0.0;
    accelMsg.linear_acceleration_covariance[3] = 0.0;
    accelMsg.linear_acceleration_covariance[4] = 0.01;
    accelMsg.linear_acceleration_covariance[5] = 0.0;
    accelMsg.linear_acceleration_covariance[6] = 0.0;
    accelMsg.linear_acceleration_covariance[7] = 0.0;
    accelMsg.linear_acceleration_covariance[8] = 0.01;

    accelMsg.angular_velocity.x = (accData[0])/64.0*9.81;
    accelMsg.angular_velocity.y = (accData[1])/64.0*9.81;
    accelMsg.angular_velocity.z = 0;
    accelMsg.angular_velocity_covariance[0] = 0.01;
    accelMsg.angular_velocity_covariance[1] = 0.0;
    accelMsg.angular_velocity_covariance[2] = 0.0;
    accelMsg.angular_velocity_covariance[3] = 0.0;
    accelMsg.angular_velocity_covariance[4] = 0.01;
    accelMsg.angular_velocity_covariance[5] = 0.0;
    accelMsg.angular_velocity_covariance[6] = 0.0;
    accelMsg.angular_velocity_covariance[7] = 0.0;
    accelMsg.angular_velocity_covariance[8] = 0.01;
    geometry_msgs::Quaternion odomQuat1 = tf::createQuaternionMsgFromYaw(0);
    accelMsg.orientation = odomQuat1;
    accelMsg.orientation_covariance[0] = 0.01;
    accelMsg.orientation_covariance[1] = 0.0;
    accelMsg.orientation_covariance[2] = 0.0;
    accelMsg.orientation_covariance[3] = 0.0;
    accelMsg.orientation_covariance[4] = 0.01;
    accelMsg.orientation_covariance[5] = 0.0;
    accelMsg.orientation_covariance[6] = 0.0;
    accelMsg.orientation_covariance[7] = 0.0;
    accelMsg.orientation_covariance[8] = 0.01;
    accelPublisher[tag].publish(accelMsg);
}

void Robot::IIR_filter(){
    
    // C2 * y(n) = (c1 * y(n-1) + x(n) + x(n-1))
    int c1 = 15, c2 = 17;

    accx_filtered = c1 * accx_filtered_prev + accx_raw_measure + accx_raw_measure_prev;
    accx_filtered = accx_filtered / c2;

    accx_filtered_prev = accx_filtered;
    accx_raw_measure_prev = accx_raw_measure;

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

        br.sendTransform(robots_dict[it->first].odomTrans);
        br.sendTransform(robots_dict[it->first].odomTransInit);
	}
}


int getIdFromAddress(int address) {
    std::map<int, Robot>::iterator it;
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        if(robots_dict[it->first].address == address) {
            return int(it->first);
        }
    }
    return -1;
}

void handlerAllOpti(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int nr_robots = int(msg->data[0]);
    int tag = 0;
    for(int i = 0; i < nr_robots; ++i){
        tag = i;
        cameras_dict[tag].cam_x = double(msg->data[i*3 + 1]);
        cameras_dict[tag].cam_y = double(msg->data[i*3 + 2]);;
        cameras_dict[tag].cam_phi = double(msg->data[i*3 + 3]);;
    }

}

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

            robots_dict[tag].accelMsg.angular_velocity.x = double(msg->data[i*5+3]);
            robots_dict[tag].accelMsg.angular_velocity.y = double(msg->data[i*5+4]);

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
    }
}

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
    }
}

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
    }
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
    
    char buff[6];

	std::map<int, Robot>::iterator it;

	for (it = robots_dict.begin(); it != robots_dict.end(); it++){
	    bool *is_sensor_active = get_sensors_condition(robots_dict[it->first].proxData);

        if(robots_dict[it->first].changedActuators[MOTORS]){
            robots_dict[it->first].changedActuators[MOTORS] = false;
            setLeftSpeed(robots_dict[it->first].address, robots_dict[it->first].speedLeft);
            setRightSpeed(robots_dict[it->first].address, robots_dict[it->first].speedRight);
		}

        if(robots_dict[it->first].changedActuators[BLUE_LEDS] and
        robots_dict[it->first].changedActuators[RED_LEDS] and
        robots_dict[it->first].changedActuators[GREEN_LEDS]) {
            robots_dict[it->first].changedActuators[RED_LEDS] = false;
            robots_dict[it->first].changedActuators[BLUE_LEDS] = false;
            robots_dict[it->first].changedActuators[GREEN_LEDS] = false;
            setAllColors(robots_dict[it->first].address, robots_dict[it->first].redLed,
                         robots_dict[it->first].greenLed, robots_dict[it->first].blueLed);
        }


        if(robots_dict[it->first].changedActuators[BLUE_LEDS_MOVE] and
            robots_dict[it->first].changedActuators[RED_LEDS_MOVE] and
            robots_dict[it->first].changedActuators[GREEN_LEDS_MOVE]) {
            robots_dict[it->first].changedActuators[RED_LEDS_MOVE] = false;
            robots_dict[it->first].changedActuators[BLUE_LEDS_MOVE] = false;
            robots_dict[it->first].changedActuators[GREEN_LEDS_MOVE] = false;
            setAllColors(robots_dict[it->first].address, robots_dict[it->first].redLed_move,
                         robots_dict[it->first].greenLed_move, robots_dict[it->first].blueLed_move);

            trigger_motive = true;

        }

        if(robots_dict[it->first].changedActuators[GREEN_LEDS_RESET]){
            robots_dict[it->first].changedActuators[GREEN_LEDS_RESET] = false;
            setGreen(robots_dict[it->first].address, robots_dict[it->first].greenLed_reset);
        }

        // setLeftSpeed(robots_dict[it->first].address, 15);
        // setRightSpeed(robots_dict[it->first].address, 15);
	}



}


// void marix_multiply(double A){


    
// }


void main_fuc(std::map<int, Robot> robots_dict, std::map<int, Camera> cameras_dict){
    
    
    
    std::map<int, Robot>::iterator it;
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        std::cout << "[" << nodeName << "] " << "accData[0]: " << robots_dict[it->first].accData[0] << std::endl;
        // std::cout << robots_dict[it->first].xPos << std::endl;

        // for each robot
        // robots_dict[it->first].timer += 1;
        // robots_dict[it->first].xPosc += robots_dict[it->first].accData[0]/64.0*9.81 * 0.001;
        // robots_dict[it->first].yPosc += robots_dict[it->first].accData[1]/64.0*9.81 * 0.001;
        // // if(robots_dict[it->first].tag == 0){
        // //     std::cout << "[" << nodeName << "] " << "accData[0]: " << robots_dict[it->first].xPosc << std::endl;
        // // }
        // std::cout << "[" << nodeName << "] " << "accData[0]: " << robots_dict[it->first].xPosc << std::endl;
        // if(robots_dict[it->first].tag == 1){
        //     std::cout << ros::Time::now() << std::endl;
        // }
        

        // 1.take the measurement from the odom and cam

        // 2.estimate the position

        // 3.decide where to go

        // 4.execution
        

    }
    // std::map<int, Camera>::iterator it1;
    // for (it1 = cameras_dict.begin(); it1 != cameras_dict.end(); it1++){
    //     std::cout << "[" << nodeName << "] " << "camera x: " << cameras_dict[it1->first].cam_x << std::endl;
    // }
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
    XmlRpc::XmlRpcValue body_list;

	np.param<std::string>("base_tag", baseTag, "elisa3");
	np.param<std::string>("name", nodeName, "elisa3");
    np.param("rigid_bodies", body_list, body_list);
    np.param("accelerometer", enabledSensors[ACCELEROMETER], false);
    np.param("floor", enabledSensors[FLOOR], false);
    np.param("proximity", enabledSensors[PROXIMITY], false);
    np.param("motor_position", enabledSensors[MOTOR_POSITION], false);

	int N_robots = body_list.size();
	int robot_addresses[N_robots];

    if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0){
		XmlRpc::XmlRpcValue::iterator i;
		
		int count = 0;
		for (i = body_list.begin(); i != body_list.end(); ++i) {
//		    std::cout << "[" << nodeName << "] " << "init number:: " << count << std::endl;
			Robot class_inst;

            // Camera camera_init;

            // camera_init.tag = count;
            
            // camera_init.OptiSubscriber = n.subscribe(("Bebop"+ std::to_string(camera_init.tag) + "/ground_pose"), 10, camera_init.handlerOpti)

            // cameras_dict[camera_init.tag] = camera_init;



			class_inst.speedLeft =0;
			class_inst.speedRight = 0;
			class_inst.greenLed = 0;

            class_inst.count = count;
            class_inst.address = (i->second)["address"];
//            class_inst.currentTime = ros::Time::now();

            class_inst.xPos = (i->second)["xPos"];
            class_inst.yPos = (i->second)["yPos"];
            class_inst.theta = 0.0;

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

            robot_addresses[count] = class_inst.address;

            std::stringstream ss;
            ss << "elisa3_robot_" << class_inst.tag <<"/odom";
            odomPublisher[class_inst.tag] = np.advertise<nav_msgs::Odometry>(ss.str(), 1);

            std::stringstream ss1;
            ss1 << "elisa3_robot_" << class_inst.tag <<"/accel";
            accelPublisher[class_inst.tag] = np.advertise<sensor_msgs::Imu>(ss1.str(), 1);
            
            AllLedSubscriber = n.subscribe("elisa3_all_robots/leds", 10, handlerAllLeds);

            AllResetSubscriber = n.subscribe("elisa3_all_robots/reset", 10, handlerAllReset);

            AllAutoMotiveSubscriber = n.subscribe("elisa3_all_robots/auto_motive", 10, handlerAllAutoMove);

            AllOptiSubscriber = n.subscribe("elisa3_all_robots/cams", 10, handlerAllOpti);

			count += 1;

		}
	}

    startCommunication(robot_addresses, N_robots);


    std::map<int, Robot>::iterator it;
    for (it = robots_dict.begin(); it != robots_dict.end(); it++){
        enableObstacleAvoidance(robots_dict[it->first].address);
        setXpos(robots_dict[it->first].address, robots_dict[it->first].xPos);
        setYpos(robots_dict[it->first].address, robots_dict[it->first].yPos);

    }

    // std::map<int, Camera>::iterator it;
    // for (it = robots_dict.begin(); it != robots_dict.end(); it++){
    //     enableObstacleAvoidance(robots_dict[it->first].address);
    //     setXpos(robots_dict[it->first].address, robots_dict[it->first].xPos);
    //     setYpos(robots_dict[it->first].address, robots_dict[it->first].yPos);

    // }

    while (ros::ok()) {
        updateSensorsData();
        updateRosInfo();
        updateActuators();

        // main part

        main_fuc(robots_dict, cameras_dict);
        

        ros::spinOnce();
    }

		stopCommunication();
}




