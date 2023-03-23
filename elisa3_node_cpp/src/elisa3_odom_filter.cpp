//
// Created by steven on 16-07-21.
//
#include "elisa3_odom_filter.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

std::map<int, ros::Publisher> odomPublishers;

class Robot{
public:
    int count, address;
    int tag;

    nav_msgs::Odometry odomMsg;

};

std::map<int, Robot> robots_dict;

int main(int argc,char *argv[]) {
//		double init_xpos, init_ypos, init_theta;
    unsigned char sensorsEnabled = 0;
    int i = 0;

    ros::init(argc, argv, "elisa3_odom_filter");

    ros::NodeHandle np("~"); // Private.
    ros::NodeHandle n; // Public.

    XmlRpc::XmlRpcValue body_list;
    np.param("rigid_bodies", body_list,body_list);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    int N_robots = body_list.size();
    int robot_addresses[N_robots];

    if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0) {
        XmlRpc::XmlRpcValue::iterator i;

        int count = 0;
        for (i = body_list.begin(); i != body_list.end(); ++i) {
            Robot class_inst;
            class_inst.address = (i->second)["address"];
            class_inst.tag = atoi((i->first).c_str());
            robots_dict[class_inst.tag] = class_inst;

            odomPublishers[class_inst.tag] = n.advertise<nav_msgs::Odometry>(
                    "elisa3_robot_"+ std::to_string(class_inst.tag) +"/odom", 10);
        }
    }

    int count =0;
    while (n.ok()){
        std::map<int, Robot>::iterator it;
        for (it = robots_dict.begin(); it != robots_dict.end(); it++) {
            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = tfBuffer.lookupTransform("world",
                               "elisa3_robot_"+ std::to_string(robots_dict[it->first].tag),ros::Time(0));
                tf::Quaternion q(
                        transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                robots_dict[it->first].odomMsg.pose.pose.position.x = transformStamped.transform.translation.x;
                robots_dict[it->first].odomMsg.pose.pose.position.y = transformStamped.transform.translation.y;
                robots_dict[it->first].odomMsg.pose.pose.position.z = yaw;

                robots_dict[it->first].odomMsg.pose.pose.orientation.x = transformStamped.transform.rotation.x;
                robots_dict[it->first].odomMsg.pose.pose.orientation.y = transformStamped.transform.rotation.y;
                robots_dict[it->first].odomMsg.pose.pose.orientation.z = transformStamped.transform.rotation.z;
                robots_dict[it->first].odomMsg.pose.pose.orientation.w = transformStamped.transform.rotation.w;

            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            odomPublishers[it->first].publish(robots_dict[it->first].odomMsg);
        }

        ros::spinOnce();

        ++count;

    }
}
//
//
//int main(int argc,char *argv[]) {
//    ros::init(argc, argv, "elisa3_odom_filter");
//
//    ros::NodeHandle node;
//
//    ros::Publisher filtered_odom = node.advertise<geometry_msgs::TransformStamped>("test_broadcast", 10);
//
//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);
//
//    ros::Rate loop_rate(10);
//    int count =0;
//
//    while (node.ok()){
//        geometry_msgs::TransformStamped transformStamped;
//
//        try{
//            transformStamped = tfBuffer.lookupTransform("world","elisa3_robot_2",ros::Time(0));
//        }
//        catch (tf2::TransformException &ex) {
//            ROS_WARN("%s", ex.what());
//            ros::Duration(1.0).sleep();
//            continue;
//        }
//        filtered_odom.publish(transformStamped);
//        ros::spinOnce();
//
//        loop_rate.sleep();
//        ++count;
//
//    }
//    return 0;
//}
