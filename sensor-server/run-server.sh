#!/bin/bash
FILE="config.ini"
COPYRIGHT="Powered by D5\n(c)2019 Lat-d5.com Ltd"
function advancedMenu() {
    ADVSEL=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --menu "Which USB-CAN device are you using?" 15 60 5 \
        "1" "CANable/CANable Pro (@can0)" \
        "2" "PCAN (Peak) (@PCAN_USBBUS1)" \
        "3" "esd CAN-USB/2 (@can0)" \
        "4" "VScom USB-CAN Plus (@slcan0 on /dev/ttyUSB0)" \
        "5" "Other (manual config)" 3>&1 1>&2 2>&3)
    case $ADVSEL in
        1)
            echo "CANable"
            echo "bustype = socketcan" >> $FILE
            echo "channel = can0" >> $FILE
            sudo ip link set can0 type can bitrate 1000000
            sudo ip link set up can0
        ;;
        2)
            echo "PCAN"
            echo "bustype = pcan" >> $FILE
            echo "channel = PCAN_USBBUS1" >> $FILE
        ;;
        3)
            echo "esd"
            echo "bustype = socketcan" >> $FILE
            echo "channel = can0" >> $FILE
            sudo ip link set can0 type can bitrate 1000000
            sudo ip link set up can0
        ;;
        4)
            echo "VScom"
            echo "bustype = socketcan" >> $FILE
            echo "channel = slcan0" >> $FILE
            sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
            sudo ifconfig slcan0 up
        ;;
        5)
            echo "other"
            ADVSEL2=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --menu "Which BUS your device uses?" 15 60 4 \
                "1" "socketcan" \
                "2" "slcan" \
                "3" "serial" \
                "4" "pcan" 3>&1 1>&2 2>&3)
            case $ADVSEL2 in
                1)
                    echo "socketcan"
                    echo "bustype = socketcan" >> $FILE
                    CBT=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --inputbox "Please set the Channel" 15 60 "can0" 3>&1 1>&2 2>&3)
                    echo "channel = $CBT" >> $FILE
                    sudo ip link set $CBT type can bitrate 1000000
                    sudo ip link set up $CBT
                ;;
                2)
                    echo "slcan"
                    echo "bustype = socketcan" >> $FILE
                    CBT=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --inputbox "Please set the Channel" 15 60 "slcan0" 3>&1 1>&2 2>&3)
                    CBD=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --inputbox "Please set the Device" 15 60 "/dev/ttyUSB0" 3>&1 1>&2 2>&3)
                    echo "channel = $CBT" >> $FILE
                    sudo slcand -o -s8 -t hw -S 3000000 $CBD
                    sudo ifconfig $CBT up
                ;;
                3)
                    echo "serial"
                    echo "bustype = serial" >> $FILE
                    CBT=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --inputbox "Please set the Channel" 15 60 "/dev/serial" 3>&1 1>&2 2>&3)
                    echo "channel = $CBT" >> $FILE
                ;;
                4)
                    echo "pcan"
                    echo "bustype = pcan" >> $FILE
                    CBT=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --inputbox "Please set the Channel" 15 60 "PCAN_USBBUS1" 3>&1 1>&2 2>&3)
                    echo "channel = $CBT" >> $FILE
                ;;
            esac
        ;;
    esac
}
AD=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --menu "Would you like to configure your system now?" 15 60 2 \
    "1" "no" \
    "2" "yes" \
    3>&1 1>&2 2>&3)
case $AD in
    1)
        echo "Server starting..."
    ;;
    2)
        echo "[CAN]" > $FILE
        advancedMenu
        echo "[controller]" >> $FILE
        SDA=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT"  --menu "Please select your sensor(s)" 15 60 4\
            "1" "4x4"\
            "2" "4x6"\
            "3" "4x8"\
            3>&1 1>&2 2>&3)
        case $SDA in
            1)
                NSDA=4
                NCHP=4
            ;;
            2)
                NSDA=4
                NCHP=6
            ;;
            3)
                NSDA=4
                NCHP=8
            ;;
        esac
        echo "num_sda = $NSDA" >> $FILE
        echo "num_chip = $NCHP" >> $FILE
        BRD=$(whiptail --title "Xela Configurator" --fb --backtitle "$COPYRIGHT" --inputbox "Please set the number of sensors on CAN" 15 60 "1" 3>&1 1>&2 2>&3)
        echo "num_brd = $BRD" >> $FILE
        echo "id_base = 512" >> $FILE
        echo "[viz]" >> $FILE
        echo "max_offset = 20" >> $FILE
        echo "max_size = 50" >> $FILE
        whiptail --title "End" --msgbox "Configuration is complete" 8 45
    ;;
esac
python server.py