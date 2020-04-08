# CAN-USB
Code for XELA sensors

## Latest version: 1.4.0

>Note: Changes are major and it is incompatible with older services

>Note2: ROS version (node to translate info from SocketIO to ROS message) is coming soon

Files:
| Name | Purpose |
| --- | --- |
| XELA_Conf.exe<br>xela_conf | Sensor auto-configuration tool |
| XELA_Log.exe<br>xela_log | Sensor logging tool |
| XELA_Server.exe<br>xela_server | Sensor Server executable |
| XELA_Viz.exe<br>xela_viz | Sensor visualizer |

>Note: more info in the Manual inside the release folder

List of suitable USB-CAN controllers:
>Note: Windows is very unreliable with the devices and support is minimal

| Status | Device | Bus (Linux) | Channel (Linux) | Details |
| --- | --- | --- | --- | --- |
| ![][~LinOK]<br>![][~WinOK] | esd CAN-USB/2 | socketcan | can0 | Network must be pulled up manually in Linux<br>Windows (bus: esd, channel: none) |
| ![][~LinOK]<br>![][~WinBad] | VSCom USB-CAN Plus | socketcan | slcan0 | Network must be pulled up manually in Linux<br>Windows (bus: slcan, channel: COMx@3000000) (x is for COM port number) |
| ![][~LinOK]<br>![][~WinSoSo] | CANable/CANable Pro | socketcan | can0 | 'candlelight' firmware required<br>Network must be pulled up manually<br>N/A in Windows |
| ![][~LinOK]<br>![][~WinOK] | PEAK USB-CAN | pcan | CAN_USBBUS1 | Same for Windows |

Setup instructions for can0 network:
> Don't forget to flash firmware to 'candlelight' if using CANable device
```console
user@localhost:~$ sudo ip link set can0 type can bitrate 1000000
user@localhost:~$ sudo ip link set up can0
```

Setup instructions for slcan0 network:
```console
user@localhost:~$ sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
user@localhost:~$ sudo ifconfig slcan0 up
```

Please remember that the commands listed are only for guidance and might differ based on your system configuration and devices already connected

**TODO List:**
- [x] Server to support several USB-CAN adapters with single code
- [x] Visualization program for sensors
- [x] Configuration file for server and visualizer
- [ ] Installation file for server and visualizer (Windows)
- [ ] Installation file for server and visualizer (Linux)
- [x] Support for following devices:
   - [x] esd CAN-USB/2 (Linux + Windows) (focused)
   - [x] VSCan USB-CAN Plus (Linux only)
   - [x] CANable/CANable Pro with candlelight firmware (Linux + Windows)
   - [x] PCAN (Linux + Windows)
   - [x] ~~8devices USB-CAN~~ *Removed as product is discontinued*
- [ ] Configuration system for:
   - [ ] Windows
   - [x] Linux


[~WinOK]: https://img.shields.io/badge/-Windows-brightgreen
[~WinSoSo]: https://img.shields.io/badge/-Windows-yellow
[~WinBad]: https://img.shields.io/badge/-Windows-red
[~LinOK]: https://img.shields.io/badge/-Linux-brightgreen
[~LinSoSo]: https://img.shields.io/badge/-Linux-yellow
[~LinBad]: https://img.shields.io/badge/-Linux-red
