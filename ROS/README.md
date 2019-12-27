### Xela Server for ROS

In this folder will be all versions of xela server for ROS

Once file is unpacked, run following commands:
```console
user@ROS:~$ catkin_make -DCMAKE_INSTALL_PREFIX=path/to/xela_sensors_ros install
```
and
```console
user@ROS:~$ echo "source ~/catkin_ws/install/setup.bash" >> ~/.bashrc
or
user@ROS:~$ echo "source path/to/xela_sensors_ros/setup.bash" >> ~/.bashrc
```

more info at [wiki.ros.org/catkin/commands/catkin_make](http://wiki.ros.org/catkin/commands/catkin_make)

### Before you start:

>Please make sure you have the suitable Python version installed.
```console
Python 2.7
```
>The required libraries are following:
```console
numpy
matplotlib
python-can
Tkinter
psutil
```
>Optional libraries are following:
```console
requests
```
>Xela Command Centre (since version 0.0.9b)
```console
user@ROS:~$ rosrun xela_server xela
```
> The sensors need to be configured and server set up (it can be done from Xela Command Centre)
```console
user@ROS:~$ rosrun xela_server xela_conf
```
> As the configuration file needs to be located in #/etc/xela# folder, you might need to create it with correct access permissions prior to launching the configuration tool.

> Run the configuration tool
```console
user@ROS:~$ python /path/to/xela/nodes/xela_server/scripts/xConf.pyc
```

## How to use:
### Manual
> 1st, start your ROS core
```console
user@ROS:~$ roscore
```
> 2nd, start Xela server
```console
user@ROS:~$ rosrun xela_server xServer.pyc
```
> 3rd, start message service
```console
user@ROS:~$ rosrun xela_sensors xSensorService.pyc
```
> 4th, run your code or Xela Visualizer
```console
user@ROS:~$ rosrun xela_server xViz.pyc
```
### Automatic
> Run everything through Xela Command Centre
```console
user@ROS:~$ rosrun xela_server xela
```

### Available services:

| service | use case | example |
| --- | --- | --- |
| xServX | to get only X coordinate of taxel | data = srv(1,2) #Sensor 1, taxel 2 |
| xServY | to get only Y coordinate of taxel | data = srv(1,2) #Sensor 1, taxel 2 |
| xServZ | to get only Z coordinate of taxel | data = srv(1,2) #Sensor 1, taxel 2 |
| xServXY | to get only X and Y coordinates of taxel | data = srv(1,2) #Sensor 1, taxel 2 |
| xServXYZ | to get all X, Y and Z coordinates of taxel | data = srv(1,2) #Sensor 1, taxel 2 |
| xServStream | to get only X coordinate of taxel | data = srv(1) #Sensor 1 |

### Available message types:

| message name | message type | response | example |
| --- | --- | --- | --- |
| xServX | XelaSensorX | value | value: 16457 |
| xServY | XelaSensorY | value | value: 16457 |
| xServZ | XelaSensorZ | value | value: 16457 |
| xServXY | XelaSensorXY | values | values: [16457, 16553] |
| xServXYZ | XelaSensorXYZ | values | values: [16457, 16553, 32057] |
| xServStream | XelaSensorStream | xyz | xyz: [1: [16457, 16553, 32057], 2: [16775, 16958, 31886] ] |

### Example usage:
```python
#!/usr/bin/env python

import rospy

from xela_sensors.srv import XelaSensorXYZ

import sys

rospy.init_node('use_service')
#wait the service to be advertised, otherwise the service use will fail
rospy.wait_for_service('xServXYZ')

#setup a local proxy for the service (we will ask for X,Y and Z data)
srv=rospy.ServiceProxy('xServXYZ', XelaSensorXYZ)

#use the service and send it a value. In this case, I am sending sensor: 1 and taxel: 3
service_example=srv(1,3)

#print the result from the service
print(service_example)

```

### Changelog:
>0.0.9
```
[+] Add Xela Command Centre
[+] Improved node with support for more sensors
[~] Optimizations
```

>0.0.4 - 0.0.8
```
[~] Optimizations, unpublished
```

>0.0.3
```
[+] Add coloring to the output
[+] Include src folder in the package so everyone can compile on their own
[~] Optimization
[!] Made for Python 2.7 only
```

>0.0.2
```
[+] Add configuration
[+] Add Visualization
[~] Optimization
```

>0.0.1
```
[~] Initial release
```
