
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
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>

#define DEBUG_ROS_PARAMS 0
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

#define ACTUATORS_NUM 4
#define MOTORS 0
#define GREEN_LEDS 1
#define RGB_LED 2
#define IR_TX 3

#define WHEEL_DISTANCE 0.041		// Distance between wheels in meters (axis length).
#define ROBOT_RADIUS 0.025			// meters.

int robotAddress[1];
bool enabledSensors[SENSORS_NUM];
bool changedActuators[ACTUATORS_NUM];
int speedLeft = 0, speedRight = 0;
unsigned char ledNum = 0, ledState = 0;
std::string elisa3Name;
struct timeval currentTime2, lastTime2;
struct timeval currentTime3, lastTime3;

signed int accData[3];
unsigned int floorData[4];
unsigned int proxData[8];
signed int robTheta=0, robXPos=0, robYPos=0;

ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;
ros::Publisher odomPublisher;
nav_msgs::Odometry odomMsg;
ros::Publisher accelPublisher;
sensor_msgs::Imu accelMsg;
ros::Publisher floorPublisher;
visualization_msgs::Marker floorMsg;

ros::Subscriber cmdVelSubscriber;

double xPos, yPos, theta;
double robXPosPrev, robYPosPrev, robThetaPrev, robDeltaX, robDeltaY, robDeltaTheta;
double deltaXCorr, deltaYCorr;
double xPosCorr, yPosCorr;
double robDistTraveled, robDistTraveledPrev, robDeltaDistTraveled;
ros::Time currentTime, lastTime, currentTimeMap, lastTimeMap;

void updateActuators() {
    
    char buff[6];

    if(changedActuators[MOTORS]) {
        changedActuators[MOTORS] = false;
				setLeftSpeed(robotAddress[0], speedLeft);
				setRightSpeed(robotAddress[0], speedRight);
    }
    
    if(changedActuators[GREEN_LEDS]) {
        changedActuators[GREEN_LEDS] = false;
    }

}

void updateSensorsData() {

		if(enabledSensors[ACCELEROMETER]) {
				accData[0] = getAccX(robotAddress[0]);
				accData[1] = getAccY(robotAddress[0]);
				accData[2] = getAccZ(robotAddress[0]);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << elisa3Name << "] " << "acc: " << accData[0] << "," << accData[1] << "," << accData[2] << std::endl;
		}

		if(enabledSensors[FLOOR]) {
				getAllGround(robotAddress[0], floorData);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << elisa3Name << "] " << "floor: " << floorData[0] << "," << floorData[1] << "," << floorData[2] << "," << floorData[3] << std::endl;
		}

		if(enabledSensors[PROXIMITY]) {
				getAllProximity(robotAddress[0], proxData);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << elisa3Name << "] " << "prox: " << proxData[0] << "," << proxData[1] << "," << proxData[2] << "," << proxData[3] << "," << proxData[4] << "," << proxData[5] << "," << proxData[6] << "," << proxData[7] << std::endl;
		}

		if(enabledSensors[MOTOR_POSITION]) {
				robXPos = getOdomXpos(robotAddress[0]);
        robYPos = getOdomYpos(robotAddress[0]);
        robTheta = getOdomTheta(robotAddress[0]);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << elisa3Name << "] " << "position: " << robXPos << "," << robYPos << "," << robTheta << std::endl;
		}
    
}

double proxToMeters(int value) {
		// Transform the analog value to a distance value in meters (given from field tests).
		if(value <= 950) {
				return (18.0/sqrt(value)+0.5)/100.0;
		} else {
				return (-0.0192*(value)+19.34)/100.0;
		}
}

void updateRosInfo() {
    static tf::TransformBroadcaster br;
    
    int i = 0;
    if(enabledSensors[PROXIMITY]) {
				if(DEBUG_RANGE_SENSORS)std::cout << "[" << elisa3Name << "] " << "ranges: ";
        for(i=0; i<8; i++) {
            if(proxData[i] > 0) {
								proxMsg[i].range = proxToMeters(proxData[i]);
            } else {
                proxMsg[i].range = proxMsg[i].max_range;
            }
            if(proxMsg[i].range > proxMsg[i].max_range) {
                proxMsg[i].range = proxMsg[i].max_range;
            }
            if(proxMsg[i].range < proxMsg[i].min_range) {
                proxMsg[i].range = proxMsg[i].min_range;
            }
            proxMsg[i].header.stamp = ros::Time::now();
            proxPublisher[i].publish(proxMsg[i]);

						if(DEBUG_RANGE_SENSORS)std::cout << proxMsg[i].range << ",";
        }

				if(DEBUG_RANGE_SENSORS)std::cout << std::endl;

        // e-puck proximity positions (cm), x pointing forward, y pointing left
        //							P0(2.4, 0.0)
        //				P7(1.7, 1.7)		P1(1.7, -1.7)
        //		P6(0.0, 2.4)        		P2(0.0, -2.4)
        //				P5(-1.7, 1.7)		P3(-1.7, -1.7)
				//							P4(-2.4, 0.0)
        //
        // e-puck proximity orentations (degrees)
        //							P0(0)
        //				P7(45)		P1(315)
        //		P6(90)        		P2(270)
        //				P5(135)		P3(225)
				//							P4(180)
        std::stringstream parent;
        std::stringstream child;
        tf::Transform transform;
        tf::Quaternion q;
        
        transform.setOrigin( tf::Vector3(0.024, 0.0, 0.013) );        
        q.setRPY(0, 0, 0);
        transform.setRotation(q);        
        parent << elisa3Name << "/base_prox0";
        child << elisa3Name << "/base_link";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        
        transform.setOrigin( tf::Vector3(0.017, -0.017, 0.013) );        
        q.setRPY(0, 0, -0.7854);
        transform.setRotation(q);
        parent.str("");
        parent << elisa3Name << "/base_prox1";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        
        transform.setOrigin( tf::Vector3(0.000, -0.024, 0.013) );        
        q.setRPY(0, 0, -1.57075);
        transform.setRotation(q);
        parent.str("");
        parent << elisa3Name << "/base_prox2";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        
        transform.setOrigin( tf::Vector3(-0.017, -0.017, 0.013) );        
        q.setRPY(0, 0, -2.3561);
        transform.setRotation(q);
        parent.str("");
        parent << elisa3Name << "/base_prox3";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        
        transform.setOrigin( tf::Vector3(-0.024, 0.000, 0.013) );        
        q.setRPY(0, 0, -3.1415);
        transform.setRotation(q);
        parent.str("");
        parent << elisa3Name << "/base_prox4";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        
        transform.setOrigin( tf::Vector3(-0.017, 0.017, 0.013) );        
        q.setRPY(0, 0, 2.3561);
        transform.setRotation(q);
        parent.str("");
        parent << elisa3Name << "/base_prox5";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        
        transform.setOrigin( tf::Vector3(0.000, 0.024, 0.013) );        
        q.setRPY(0, 0, 1.57075);
        transform.setRotation(q);
        parent.str("");
        parent << elisa3Name << "/base_prox6";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        
        transform.setOrigin( tf::Vector3(0.017, 0.017, 0.013) );        
        q.setRPY(0, 0, 0.7854);
        transform.setRotation(q);
        parent.str("");
        parent << elisa3Name << "/base_prox7";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        

        currentTimeMap = ros::Time::now();
        parent.str("");
        parent << elisa3Name << "/base_laser";
        //populate the LaserScan message
        laserMsg.header.stamp = ros::Time::now();
        laserMsg.header.frame_id = parent.str();
        laserMsg.angle_min = -M_PI/2.0;
        laserMsg.angle_max = M_PI/2.0;
        laserMsg.angle_increment = M_PI/18.0; // 10 degrees.
        //laserMsg.time_increment = (currentTimeMap-lastTimeMap).toSec()/180; //0.003; //(1 / laser_frequency) / (num_readings);
        //laserMsg.scan_time = (currentTimeMap-lastTimeMap).toSec();
        // The laser is placed in the center of the robot, but the proximity sensors are placed around the robot thus add "ROBOT_RADIUS" to get correct values.
        laserMsg.range_min = 0.005+ROBOT_RADIUS; // 0.5 cm + ROBOT_RADIUS.
        laserMsg.range_max = 0.05+ROBOT_RADIUS; // 5 cm + ROBOT_RADIUS. 
        laserMsg.ranges.resize(19);
        laserMsg.intensities.resize(19);
        lastTimeMap = ros::Time::now();
        
        // We use the information from the 6 proximity sensors on the front side of the robot to get 19 laser scan points. The interpolation used is the following:
        // laser[0] at -90 degrees: P2
        // laser[1] at -80 degrees: 4/5*P2 + 1/5*P1
        // laser[2] at -70 degrees: 3/5*P2 + 2/5*P1
        // laser[3] at -60 degrees: 2/5*P2 + 3/5*P1
        // laser[4] at -50 degrees: 1/5*P2 + 4/5*P1
        // laser[5] at -40 degrees: P1
        // laser[6] at -30 degrees: 3/4*P1 + 1/4*P0
        // laser[7] at -20 degrees: 2/4*P1 + 2/4*P0
        // laser[8] at -10 degrees: 1/4*P1 + 3/4*P0
        // laser[9] at 0 degrees: P0
        // laser[10] at 10 degrees: 1/4*P7 + 3/4*P0
        // laser[11] at 20 degrees: 2/4*P7 + 2/4*P0
        // laser[12] at 30 degrees: 3/4*P7 + 1/4*P0
        // laser[13] at 40 degrees: P7
        // laser[14] at 50 degrees: 1/5*P6 + 4/5*P7
        // laser[15] at 60 degrees: 2/5*P6 + 3/5*P7
        // laser[16] at 70 degrees: 3/5*P6 + 2/5*P7
        // laser[17] at 80 degrees: 4/5*P6 + 1/5*P7
        // laser[18] at 90 degrees: P6
        
        float tempProx;
        tempProx = proxData[2];
        if(tempProx > 0) {   
            laserMsg.ranges[0] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[0] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[0] = laserMsg.range_max;
            laserMsg.intensities[0] = 0;
        }
  
        tempProx = proxData[2]*4/5 + proxData[1]*1/5;
        if(tempProx > 0) {   
            laserMsg.ranges[1] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[1] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[1] = laserMsg.range_max;
            laserMsg.intensities[1] = 0;
        }
        
        tempProx = proxData[2]*3/5 + proxData[1]*2/5;
        if(tempProx > 0) {   
            laserMsg.ranges[2] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[2] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[2] = laserMsg.range_max;
            laserMsg.intensities[2] = 0;
        }
        
        tempProx = proxData[2]*2/5 + proxData[1]*3/5;
        if(tempProx > 0) {   
            laserMsg.ranges[3] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[3] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[3] = laserMsg.range_max;
            laserMsg.intensities[3] = 0;
        }        
        
        tempProx = proxData[2]*1/5 + proxData[1]*4/5;
        if(tempProx > 0) {   
            laserMsg.ranges[4] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[4] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[4] = laserMsg.range_max;
            laserMsg.intensities[4] = 0;
        }        
        
        tempProx = proxData[1];
        if(tempProx > 0) {   
            laserMsg.ranges[5] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[5] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[5] = laserMsg.range_max;
            laserMsg.intensities[5] = 0;
        }        
        
        tempProx = proxData[1]*3/4 + proxData[0]*1/4;
        if(tempProx > 0) {   
            laserMsg.ranges[6] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[6] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[6] = laserMsg.range_max;
            laserMsg.intensities[6] = 0;
        }        
        
        tempProx = proxData[1]*2/4 + proxData[0]*2/4;
        if(tempProx > 0) {   
            laserMsg.ranges[7] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[7] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[7] = laserMsg.range_max;
            laserMsg.intensities[7] = 0;
        }         
        
        tempProx = proxData[1]*1/4 + proxData[0]*3/4;
        if(tempProx > 0) {   
            laserMsg.ranges[8] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[8] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[8] = laserMsg.range_max;
            laserMsg.intensities[8] = 0;
        }         
        
        tempProx = proxData[0];
        if(tempProx > 0) {   
            laserMsg.ranges[9] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[9] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[9] = laserMsg.range_max;
            laserMsg.intensities[9] = 0;
        }         
        
        tempProx = proxData[7]*1/4 + proxData[0]*3/4;
        if(tempProx > 0) {   
            laserMsg.ranges[10] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[10] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[10] = laserMsg.range_max;
            laserMsg.intensities[10] = 0;
        }         
        
        tempProx = proxData[7]*2/4 + proxData[0]*2/4;
        if(tempProx > 0) {   
            laserMsg.ranges[11] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[11] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[11] = laserMsg.range_max;
            laserMsg.intensities[11] = 0;
        }         
        
        tempProx = proxData[7]*3/4 + proxData[0]*1/4;
        if(tempProx > 0) {   
            laserMsg.ranges[12] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[12] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[12] = laserMsg.range_max;
            laserMsg.intensities[12] = 0;
        }         
        
        tempProx = proxData[7];
        if(tempProx > 0) {   
            laserMsg.ranges[13] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[13] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[13] = laserMsg.range_max;
            laserMsg.intensities[13] = 0;
        }         
        
        tempProx = proxData[7]*4/5 + proxData[6]*1/5;
        if(tempProx > 0) {   
            laserMsg.ranges[14] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[14] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[14] = laserMsg.range_max;
            laserMsg.intensities[14] = 0;
        }   
        
        tempProx = proxData[7]*3/5 + proxData[6]*2/5;
        if(tempProx > 0) {   
            laserMsg.ranges[15] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[15] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[15] = laserMsg.range_max;
            laserMsg.intensities[15] = 0;
        }                      
        
        tempProx = proxData[7]*2/5 + proxData[6]*3/5;
        if(tempProx > 0) {   
            laserMsg.ranges[16] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[16] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[16] = laserMsg.range_max;
            laserMsg.intensities[16] = 0;
        }          
        
        tempProx = proxData[7]*1/5 + proxData[6]*4/5;
        if(tempProx > 0) {   
            laserMsg.ranges[17] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[17] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[17] = laserMsg.range_max;
            laserMsg.intensities[17] = 0;
        }          
        
        tempProx = proxData[6];
        if(tempProx > 0) {   
            laserMsg.ranges[18] = proxToMeters(tempProx)+ROBOT_RADIUS;
            laserMsg.intensities[18] = tempProx; 
        } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
            laserMsg.ranges[18] = laserMsg.range_max;
            laserMsg.intensities[18] = 0;
        }          
        
        for(i=0; i<19; i++) {
            if(laserMsg.ranges[i] > laserMsg.range_max) {
                laserMsg.ranges[i] = laserMsg.range_max;
            }
            if(laserMsg.ranges[i] < laserMsg.range_min) {
                laserMsg.ranges[i] = laserMsg.range_min;
            }
        }
        
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.034) );        
        q.setRPY(0, 0, 0);
        transform.setRotation(q);        
        parent.str("");
        child.str("");
        parent << elisa3Name << "/base_laser";
        child << elisa3Name << "/base_link";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        
        laserPublisher.publish(laserMsg);
        
    }
    

    if(enabledSensors[MOTOR_POSITION]) {
        
				robDeltaX = robXPos - robXPosPrev;
				robDeltaY = robYPos - robYPosPrev;
				robXPosPrev = robXPos;
				robYPosPrev = robYPos;				
				theta = robTheta*M_PI/180;    // Expressed in radiant.
				// We noticed from field tests on a vertical wall that there is a difference in the measured distance between
				// a route traveled toward bottom and a route traveled toward top. For this reason we adjust the distance 
				// traveled based on the angle.
				if(robTheta <= 180 && robTheta >= 0) {
						robDistTraveled = sqrt(robDeltaX*robDeltaX + robDeltaY*robDeltaY);
						deltaXCorr = robDistTraveled*2/3*cos(theta);	// 2/3 is the magical factor found from filed tests, probably you'll need to adapt it to your surface.
						deltaYCorr = robDistTraveled*2/3*sin(theta);
						if(DEBUG_ODOMETRY)std::cout << "[" << elisa3Name << "] " << "delta corr: " << deltaXCorr << ", " << deltaYCorr << std::endl;
				} else {
						deltaXCorr = robDeltaX;
						deltaYCorr = robDeltaY;
						if(DEBUG_ODOMETRY)std::cout << "[" << elisa3Name << "] " << "delta not corr: " << deltaXCorr << ", " << deltaYCorr << std::endl;
				}
				xPos += deltaXCorr/1000.0;	// Expressed in meters.
				yPos += deltaYCorr/1000.0;	// Expressed in meters.

				//xPos = robXPos/1000.0;   // Expressed in meters.
        //yPos = robYPos/1000.0;   // Expressed in meters.
        
        if(DEBUG_ODOMETRY)std::cout << "[" << elisa3Name << "] " << "x, y, theta: " << xPos << ", " << yPos << ", " << theta << std::endl;
        
				robDeltaTheta = (robTheta - robThetaPrev)*M_PI/180;
				robThetaPrev = robTheta;
				robDeltaDistTraveled = (robDistTraveled - robDistTraveledPrev)/1000.0;
				robDistTraveledPrev = robDistTraveled;

        // Publish the odometry message over ROS.
        odomMsg.header.stamp = ros::Time::now();
        odomMsg.header.frame_id = "odom";
        std::stringstream ss;
        ss << elisa3Name << "/base_link";
        odomMsg.child_frame_id = ss.str();
        odomMsg.pose.pose.position.x = xPos;       
        odomMsg.pose.pose.position.y = yPos;
        odomMsg.pose.pose.position.z = 0;
        // Since all odometry is 6DOF we'll need a quaternion created from yaw.
        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
        odomMsg.pose.pose.orientation = odomQuat;
        currentTime = ros::Time::now();
        odomMsg.twist.twist.linear.x = robDeltaDistTraveled / ((currentTime-lastTime).toSec());   // "robDeltaDistTraveled" is the linear distance covered in meters from the last update (delta distance);
                                                                                        // the time from the last update is measured in seconds thus to get m/s we multiply them.
        odomMsg.twist.twist.angular.z = robDeltaTheta / ((currentTime-lastTime).toSec());  // "robDeltaTheta" is the angular distance covered in radiant from the last update (delta angle);
                                                                                        // the time from the last update is measured in seconds thus to get rad/s we multiply them.
        if(DEBUG_ODOMETRY)std::cout << "[" << elisa3Name << "] " << "time elapsed = " << (currentTime-lastTime).toSec() << " seconds" << std::endl;
        lastTime = ros::Time::now();

        odomPublisher.publish(odomMsg);
        
        // Publish the transform over tf.
        geometry_msgs::TransformStamped odomTrans;
        odomTrans.header.stamp = odomMsg.header.stamp;
        odomTrans.header.frame_id = odomMsg.header.frame_id;
        odomTrans.child_frame_id = odomMsg.child_frame_id;
        odomTrans.transform.translation.x = xPos;
        odomTrans.transform.translation.y = yPos;
        odomTrans.transform.translation.z = 0.0;
        odomTrans.transform.rotation = odomQuat;
        br.sendTransform(odomTrans);
    }
    
    if(enabledSensors[ACCELEROMETER]) {
        std::stringstream ss;
        ss << elisa3Name << "/base_link";
        accelMsg.header.frame_id = ss.str();
        accelMsg.header.stamp = ros::Time::now();            
        accelMsg.linear_acceleration.x = (accData[0])/64.0*9.81; // 1 g = 64, then transforms in m/s^2.
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
        if(DEBUG_ACCELEROMETER)std::cout << "[" << elisa3Name << "] " << "accel raw: " << accData[0] << ", " << accData[1] << ", " << accData[2] << std::endl;
        if(DEBUG_ACCELEROMETER)std::cout << "[" << elisa3Name << "] " << "accel (m/s2): " << ((accData[0])/64.0*9.81) << ", " << ((accData[1])/64.0*9.81) << ", " << ((accData[2]+64.0)/64.0*9.81) << std::endl;
        accelMsg.angular_velocity.x = 0;
        accelMsg.angular_velocity.y = 0;
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
        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
        accelMsg.orientation = odomQuat;
        accelMsg.orientation_covariance[0] = 0.01;
        accelMsg.orientation_covariance[1] = 0.0;
        accelMsg.orientation_covariance[2] = 0.0;
        accelMsg.orientation_covariance[3] = 0.0;
        accelMsg.orientation_covariance[4] = 0.01;
        accelMsg.orientation_covariance[5] = 0.0;
        accelMsg.orientation_covariance[6] = 0.0;
        accelMsg.orientation_covariance[7] = 0.0;
        accelMsg.orientation_covariance[8] = 0.01;
        accelPublisher.publish(accelMsg);
    }
    
    if(enabledSensors[FLOOR]) {
        std::stringstream ss;
        ss << elisa3Name << "/base_link";
        floorMsg.header.frame_id = ss.str();
        floorMsg.header.stamp = ros::Time::now();
        floorMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        floorMsg.pose.position.x = 0.15;
        floorMsg.pose.position.y = 0;
        floorMsg.pose.position.z = 0.13;
        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
        floorMsg.pose.orientation = odomQuat;
        floorMsg.scale.z = 0.01;
        floorMsg.color.a = 1.0;
        floorMsg.color.r = 1.0;
        floorMsg.color.g = 1.0;
        floorMsg.color.b = 1.0;
        ss.str("");
        ss << "floor: [" << floorData[0] << ", " << floorData[1] << ", " << floorData[2] << ", " << floorData[3] << "]";
        floorMsg.text = ss.str();
        floorPublisher.publish(floorMsg);
    }

    
}

void handlerVelocity(const geometry_msgs::Twist::ConstPtr& msg) {
    // Controls the velocity of each wheel based on linear and angular velocities.
    double linear = msg->linear.x;		// Expect the linear speed to be given in cm/s.
    double angular = msg->angular.z;	// Expect the angular speed to be given in rad/s.

		if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name << "] " << "linear: " << linear << ", angular: " << angular << std::endl;

    // Kinematic model for differential robot.
    double wl = (linear - ((WHEEL_DISTANCE*100.0) / 2.0) * angular);	// Result is cm/s.
    double wr = (linear + ((WHEEL_DISTANCE*100.0) / 2.0) * angular);	// Result is cm/s.

		if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name << "] " << "kinematic: " << wl << ", " << wr << std::endl;

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
    changedActuators[MOTORS] = true;

    if(DEBUG_SPEED_RECEIVED)std::cout << "[" << elisa3Name << "] " << "new speed: " << speedLeft << ", " << speedRight << std::endl;
    
}

int main(int argc,char *argv[]) {
		double init_xpos, init_ypos, init_theta;   
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
    
    np.param("elisa3_address", robotAddress[0], 1111);
    np.param<std::string>("elisa3_name", elisa3Name, "elisa3");
    np.param("xpos", init_xpos, 0.0);
    np.param("ypos", init_ypos, 0.0);
    np.param("theta", init_theta, 0.0);
    np.param("accelerometer", enabledSensors[ACCELEROMETER], false);
    np.param("floor", enabledSensors[FLOOR], false);
    np.param("proximity", enabledSensors[PROXIMITY], false);
    np.param("motor_position", enabledSensors[MOTOR_POSITION], false); 


    if(DEBUG_ROS_PARAMS) {
        std::cout << "[" << elisa3Name << "] " << "elisa3 address: " << robotAddress[0] << std::endl;
        std::cout << "[" << elisa3Name << "] " << "elisa3 name: " << elisa3Name << std::endl;
        std::cout << "[" << elisa3Name << "] " << "init pose: " << init_xpos << ", " << init_ypos << ", " << theta << std::endl;
        std::cout << "[" << elisa3Name << "] " << "accelerometer enabled: " << enabledSensors[ACCELEROMETER] << std::endl;
        std::cout << "[" << elisa3Name << "] " << "floor enabled: " << enabledSensors[FLOOR] << std::endl;
        std::cout << "[" << elisa3Name << "] " << "proximity enabled: " << enabledSensors[PROXIMITY] << std::endl;
        std::cout << "[" << elisa3Name << "] " << "motor position enabled: " << enabledSensors[MOTOR_POSITION] << std::endl;
    }

		startCommunication(robotAddress, 1);
    

    if(enabledSensors[ACCELEROMETER]) {
				sensorsEnabled++;
        accelPublisher = n.advertise<sensor_msgs::Imu>("accel", 10);
    }
    if(enabledSensors[FLOOR]) {
				sensorsEnabled++;
        floorPublisher = n.advertise<visualization_msgs::Marker>("floor", 10);
    }
    if(enabledSensors[PROXIMITY]) {
				sensorsEnabled++;
        for(i=0; i<8; i++) {
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
            std::stringstream ss;
            ss.str("");
            ss << "proximity" << i;
            proxPublisher[i] = n.advertise<sensor_msgs::Range>(ss.str(), 10);
            //proxMsg[i] = new sensor_msgs::Range();
            proxMsg[i].radiation_type = sensor_msgs::Range::INFRARED;
            ss.str("");
            ss << elisa3Name << "/base_prox" << i;
            proxMsg[i].header.frame_id =  ss.str();
            proxMsg[i].field_of_view = 0.26;    // About 15 degrees...to be checked!
            proxMsg[i].min_range = 0.005;       // 0.5 cm.
            proxMsg[i].max_range = 0.05;        // 5 cm.                    
        }       
        
        laserPublisher = n.advertise<sensor_msgs::LaserScan>("scan", 10);
    }
    if(enabledSensors[MOTOR_POSITION]) {
				sensorsEnabled++;
        odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 10);
        currentTime = ros::Time::now();
        lastTime = ros::Time::now();        
    }

    if(sensorsEnabled == 0) {
        std::cerr << "[" << elisa3Name << "] " << "No sensors enabled!" << std::endl;
				stopCommunication();
        return -1;
    }
       
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
    cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, handlerVelocity);    
    
    theta = init_theta;
    xPos = init_xpos;
    yPos = init_ypos;

		//enableObstacleAvoidance(robotAddress[0]);
		//enableTVRemote(robotAddress[0]);

    while (ros::ok()) {
        updateSensorsData();
        updateRosInfo();
        updateActuators();
        ros::spinOnce();
				if(waitForUpdate(robotAddress[0], 10000000)) { // Wait for at most 10 seconds.
						break; // We have connection problems, stop here.
				}
    }

		stopCommunication();
}



