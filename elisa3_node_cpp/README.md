# elisa3_node_cpp

# ROS node for Elisa-3 Robot
Elisa-3 ROS node based on roscpp; it support all the elisa-3 sensors.

This package is maintained by [GCtronic](http://www.gctronic.com/).

## How to use
For detailed informations about the node refer to the [elisa-3 ROS wiki](http://www.gctronic.com/doc/index.php/Elisa-3#ROS).

source devel/setup.bash


# normal step for running the experiment

prepare for the camera connection.

1. use the ethernet cable in the lab to connect the Optitrack system.

2. open the "motive" on lab's comupter, and in data streaming pannel, filling in the ip address (usually 192.168.1.123) of your ROS computer

prepare for robot setup

1. place the robot in the lab field, and record the x, y, z data for initialization.

2. filling in those information in following files:

a. src/mocap_optitrack/config/mocap.yaml
b. src/mocap_optitrack/config/mocap_multidrone.yaml
c. src/elisa3_node_cpp/config/robot.yaml

strat the expriment

1. roscore

2. roslaunch elisa3_node_cpp elisa3_swarm.launch

3. make sure you finish the preset of the camera. Then run the camera with 

roslaunch mocap_optitrack mocap_multidrone.launch






## good robots List

3735
3769
3795
3722()

## no good motor
3762
3533
3792
3734(right motor)
3809(right motor)
3739(left motor)
3810(l)
3815


## broken motor
3783

## flush error
3755
3656
3771
3755


