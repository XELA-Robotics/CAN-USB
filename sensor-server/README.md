# CAN-USB
Code for Xela sensors

Server file is _server.py(c)_\
Visualization file is _visualizaer.py(c)_\
Configuration file is _config.ini_ 
> running _configurator.py(c)_ will help to set up proper config file
Files required for pyhton-can (with specific USB-CAN adapters)
```
PCANBasic.dll
usb2can.dll
usb2can.lib
```

List of suitable USB-CAN controllers:
>Note: Windows is very unreliable with the devices and support is minimal

| Status | Device | Bus (Linux) | Channel (Linux) | Details |
| --- | --- | --- | --- | --- |
| ![][~LinOK]<br>![][~WinSoSo] | esd CAN-USB/2 | socketcan | can0 | Network must be pulled up manually in Linux<br>Windows (bus: esd, channel: none) |
| ![][~LinOK]<br>![][~WinBad] | VSCom USB-CAN Plus | socketcan | slcan0 | Network must be pulled up manually in Linux<br>Windows (bus: slcan, channel: COMx@3000000) (x is for COM port number) |
| ![][~LinOK]<br>![][~WinBad] | CANable/CANable Pro | socketcan | can0 | 'candlelight' firmware required<br>Network must be pulled up manually<br>N/A in Windows |
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

Timings (ms/120 reads):

| Controller | OS | bus | [1] | [2] | [3] | [4] | [5] | Avg |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| esd CAN-USB/2 | Windows 10 | esd | 1109 | 1110 | 1164 | 1138 | 1170 | 1138 |
| PCAN | Windows 10 | pcan | 312 | 345 | 317 | 324 | 326 | 324 |
| esd CAN-USB/2 | Linux | socketcan | 295 | 297 | 294 | 300 | 294 | 296 |
| VScan USB-CAN Plus | Linux | socketcan | 297 | 300 | 300 | 295 | 296 | 298 |
| PCAN | Linux | pcan | 299 | 299 | 298 | 299 | 299 | 299 |
| CANable | Linux | socketcan | 298 | 298 | 294 | 295 | 295 | 296 |

>Based on the testings it looks that ''socketcan'' is the best can bus driver to use as it supports so many devices (Linux only).\
>Also ''pcan'' works properly with both Windows and Linux

**TODO List:**
- [x] Server to support several USB-CAN adapters with single code
- [x] Visualization program for sensors
- [x] Configuration file for server and visualizer
- [ ] Installation file for server and visualizer (Windows)
- [ ] Installation file for server and visualizer (Linux)
- [x] Support for following devices:
   - [x] esd CAN-USB/2 (Linux + Windows)
   - [x] VSCan USB-CAN Plus (Linux only)
   - [x] CANable/CANable Pro with candlelight firmware (Linux only)
   - [x] PCAN (Linux + Windows)
   - [x] ~~8devices USB-CAN~~ *Removed as product is discontinued*
- [x] Configuration system for:
   - [x] Windows
   - [x] Linux


[~WinOK]: https://img.shields.io/badge/-Windows-brightgreen
[~WinSoSo]: https://img.shields.io/badge/-Windows-yellow
[~WinBad]: https://img.shields.io/badge/-Windows-red
[~LinOK]: https://img.shields.io/badge/-Linux-brightgreen
[~LinSoSo]: https://img.shields.io/badge/-Linux-yellow
[~LinBad]: https://img.shields.io/badge/-Linux-red