# elisa3_node_cpp

# ROS node for Elisa-3 Robot
Elisa-3 ROS node based on roscpp; it support all the elisa-3 sensors.

This package is maintained by [GCtronic](http://www.gctronic.com/).

## How to use
For detailed informations about the node refer to the [elisa-3 ROS wiki](http://www.gctronic.com/doc/index.php/Elisa-3#ROS).

source devel/setup.bash


# normal step for running the experiment

## robots and VM

1. download the VM from this link: https://surfdrive.surf.nl/files/index.php/s/6UwWVbSCVMtV9jO, and import the VDI into virtualBox

2. replace the content in "catkin_ws/src" with the files in this github 

3. run following command:
cd catkin_ws/
catkin_make
source devel/setup.bash 

4. you might want to remake the elisa-3 lib if you change it, follow this:

Rebuild the elisa-3 library: go to ~/catkin_ws/src/elisa3_node_cpp/src/pc-side-elisa3-library/linux
then issue "make clean" then "make"


https://www.gctronic.com/doc/index.php/Elisa#1._Install_the_radio_base-station_driver



## camera
prepare for the camera connection.

1. use the ethernet cable in the lab to connect the Optitrack system.

2. open the "motive" on lab's comupter, and in data streaming pannel, filling in the ip address (usually 192.168.1.123) of your ROS computer


## experiment
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

4. go to controller_py file run
python main.py






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

3761
3790
3758
3769
3724
3815



0 1685102500.1125062
6
0 1685102500.35472
0 1685102500.3920653
1 1685102500.4392462
6
1 1685102500.4394152
1 1685102500.4404972
2 1685102500.4679396
6
2 1685102500.4681234
2 1685102500.559588
3 1685102500.5606692
6
3 1685102500.5607708
3 1685102500.5616171
4 1685102500.6526423
6
4 1685102500.6527734
4 1685102500.653645
5 1685102500.6572235
6
5 1685102500.6732106
5 1685102500.6749725
6 1685102500.8234088
6
6 1685102500.8235152
6 1685102500.8296165
7 1685102500.830301
6
7 1685102500.8304248
7 1685102501.1744099
done
