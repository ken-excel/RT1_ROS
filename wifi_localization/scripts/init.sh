#!/bin/bash
echo "Initiating system...RT1_ROS"
sudo chmod a+rx /dev/rt1 
echo "RT1 port...permission allowed"
sudo chmod 666 /dev/ttyACM0
echo "Hokuyo lidar...permission allowed"
stty -F /dev/rt1 raw -echo speed 115200
stty -F /dev/rt1 raw -echo speed 115200
echo "RT1 port speed adjusted"


