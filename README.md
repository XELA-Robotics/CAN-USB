# CAN-USB
Files in **sensor-server** folder are for setting up the sensor server and to see visually the sensor operation

List of suitable USB-CAN controllers:

| Device | Bus | Channel | Details |
| --- | --- | --- | --- |
| esd CAN-USB/2 | socketcan | can0 | Network must be pulled up manually in Linux |
| VSCom USB-CAN Plus | socketcan | slcan0 | Network must be pulled up manually in Linux |
| CANable/CANable Pro | socketcan | can0 | 'candlelight' firmware required, Network must be pulled up manually |
| PEAK USB-CAN | pcan | CAN_USBBUS1 |  |

>Named network channels (ports) are for Linux, Windows usually uses names like COM1 etc.

Setup instructions for can0 network:
> Don't forget to flash firmware to 'candlelight' if using CANable device
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

Setup instructions for slcan0 network:
```bash
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo ifconfig slcan0 up
```

Please remember that the commands listed are only for guidance and might differ based on your system configuration and devices already connected
