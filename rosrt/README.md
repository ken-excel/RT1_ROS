# Package

Robot Assist Walker RT1 

ros_start is a package of ROS node to manipulate RT.works' robot assist walker RT1

RT1 is programmed to do obstacle avoiding, passive intuitive braking system. 

# Preparation

## Hardware

Open the control box and find connector J1102.
PIN #1 and PIN #2 are for serial communication(pulled up to 3.3V), and PIN #3 is GND.

Get USB to TTL Serial Converter Cable
(For example : http://akizukidenshi.com/catalog/g/gM-05840 )
and connect those 3 pins to the cable.

**NOTE:communication lines are pulled up to 3.3v. Choose proper converter cable.**

Pin asignments are:

PIN #1 of J1102(RXD pulled up to 3.3v) --- PIN #4(ORANGE) of Converter

PIN #2 of J1102(TXD pulled up to 3.3v) --- PIN #5(YELLOW) of Converter

PIN #3 of J1102(GND) --- PIN #1(BLACK) of Converder

# Operation

## Power up the RT.2

Switch on the power on the control box. Press power button on the panel.

## On the PC

### Start roscore
	$ roscore
### Set up the serial port
```
$ stty -F /dev/ttyUSB0 raw -echo speed 115200
```
if the udev rule is already set use:
```
$ stty -F /dev/rt1 raw -echo speed 115200
```
### Start the node
```
$ rosrun ros_start rosrt_rt1
```
### Start controller
```
$ rosrun ros_start rt1_con
```
### Launch file (node, controller, odometry, rplidar)
```
$ roslaunch ros_start rt1.launch
```
