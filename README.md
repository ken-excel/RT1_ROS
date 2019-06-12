# Wifi_Navigation
# Author: Kengkij Promsutipong(k.promsutipong@srd.mech.tohoku.ac.jp)

# Package Summary

1. nav_stack (launch file and parameter for navigation stack)
2. rosrt (modified, original by RT.works)
3. rplidar_ros (modified, original by slamtec)
4. wifi_localization ()

# Troubleshoot

1. Process Killing
```
$ ps aux | grep -i apt
$ sudo kill -9 <process id>
```

2. RaspberryPi time synchronization
```
$ sudo ntpdate ntp.ubuntu.com
```

3. Install all dependencies in src (before build)
```
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

4. Check port
```
$ ls -l /dev |grep ttyUSB
$ ls -l /dev |grep ttyACM
```

5. Give permission to port (e.g. before using Lidar)
```
$ sudo chmod a+rw /dev/ttyUSB0
$ sudo chmod 666 /dev/ttyACM0
```

6. Set port speed (RT1)
```
$ stty -F /dev/ttyUSB0 raw -echo speed 115200
```
