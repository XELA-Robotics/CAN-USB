### Xela Server for ROS

In this folder will be all versions of xela server for ROS

Once file is unpacked, run following commands:
```console
user@ROS:~$ catkin_make -DCMAKE_INSTALL_PREFIX=path/to/folder install
```
and
```console
user@ROS:~$ echo "source ~/catkin_ws/install/setup.bash" >> ~/.bashrc
```

more info at [wiki.ros.org/catkin/commands/catkin_make](http://wiki.ros.org/catkin/commands/catkin_make)


Files ___4x4.ini___, ___4x6.ini___ and ___xServ.ini___ must be moved to ___/etc/xela___ folder

### Example usage:
```python
#!/usr/bin/env python
 
import rospy
 
from xela_sensors.srv import XelaSensorXYZ
 
import sys
 
rospy.init_node('use_service')
 
#wait the service to be advertised, otherwise the service use will fail
rospy.wait_for_service('xela_sensors')
 
#setup a local proxy for the service
srv=rospy.ServiceProxy('xela_sensors',XelaSensorXYZ)
 
#use the service and send it a value. In this case, I am sending sensor: 1 and taxel: 3
service_example=srv(1,3)
 
#print the result from the service
print service_example
```
