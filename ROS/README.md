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
