
#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  rt1"
echo "rt1 usb connection as /dev/rt1 , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy sonar.rules to  /etc/udev/rules.d/"
echo "`rospack find ros_start`/scripts/rt1.rules"
sudo cp `rospack find ros_start`/scripts/rt1.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
