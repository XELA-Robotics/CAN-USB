#!/usr/bin/env python

#Sensor Server
#by Matt
#release 1.1
#Support USB2CAN & PCAN & CANable

import can #main library handling the devices
import math
import time #timing purposes
import getopt
import sys
import string
import os
import thread
#import util #currently not required
import threading
import types
import csv
import socket

import serial

import ConfigParser #for INI parsing
import io #for INI parsing
import platform #for OS detection and differences

osname = platform.system()
if osname == "Windows":
    fnamewpath = os.path.realpath(__file__)
    fnamenpath = os.path.basename(fnamewpath)
    path2 = fnamewpath[0:(len(fnamewpath)-len(fnamenpath))].replace("\\","/")
    path2ini = path2 + "config.ini"
else:
    fnamewpath = os.path.realpath(__file__)
    fnamenpath = os.path.basename(fnamewpath)
    path2 = fnamewpath[0:(len(fnamewpath)-len(fnamenpath))]
    path2ini = path2 + "config.ini"
print("Open config at {}".format(path2ini))
with open(path2ini) as f: #in Windows use full path to config
    sample_config = f.read()
config = ConfigParser.RawConfigParser(allow_no_value=True)
config.readfp(io.BytesIO(sample_config))

if config.get('controller', 'id_base'):
    id_base = int(config.get('controller', 'id_base'))
else:
    id_base = 0x201 #Target ID of the MTB
if config.get('controller', 'num_sda'):
    num_sda = int(config.get('controller', 'num_sda'))
else:
    num_sda = 4 #number of sdas
if config.get('controller', 'num_chip'):
    num_of_chip = int(config.get('controller', 'num_chip'))
else:
    num_of_chip = 4 #number of chips per sda
if config.get('controller', 'num_brd'):
    num_of_board = int(config.get('controller', 'num_brd'))
else:
    num_of_board = 1 #how many controllers
if config.get('CAN', 'bustype') and config.get('CAN','channel'):
    can_bustype = config.get('CAN', 'bustype') #CAN Bus type (socketcan or pcan or slcan)
    can_channel = config.get('CAN', 'channel') #CAN port (can0 or COM6 or /dev/ttyUSB0)
else:
    print "Config not available"
    exit()

    
if osname == "Windows":
    if can_bustype == "slcan":
        print("Please be aware that SLCAN is very slow in Windows")
    elif can_bustype == "pcan":
        print("PCAN is fully supported in Windows, please make sure you have PCANBasic.dll in right system folder")
    elif can_bustype == "serial":
        print("Please be aware that Serial interface in Windows is extremely slow and lossy")
    elif can_bustype == "socketcan":
        print("socketcan is not supported in Windows")
        exit()
    elif can_bustype == "esd":
        import ntcan
        print("esd CAN-USB/2 is supported in Windows with separate thread")
    else:
        print("Windows only supports PCAN at this time")
        exit()
elif osname == "Linux":
    if can_bustype == "slcan":
        print("SLCAN can be slow")
    elif can_bustype == "socketcan" or can_bustype == "pcan":
        print("Let's connect to CAN")
    else:
        print("Unsupported Bus")
        exit()
else:
    print("Unsupported OS")
    exit()

conn = type('',(),{})() #define the empty server object in case there is failure before it is ready to be started
stop_threads = False #variable to stop all threads

cif_array = {} #can interface array
cmsg_array = {} #array for message handlers
board_start_num = 1 #first controller number
csvfile = type('',(),{})() #define the object for csvfile creation

num_taxel = num_sda * num_of_chip #total taxels per sensor

#define function to send data to controllers
def send_data(node,data,bus):
    global can_bustype
    msg = can.Message(arbitration_id=node,data=data,extended_id=False)
    try:
        bus.send(msg)
        print("Message sent on {}, data: {}".format(bus.channel_info,msg))
    except Exception,e:
        print("Message NOT sent, Error: {}".format(e))

#define function to send data to controllers
def send_data_esd(node,data,bus,msg):
    try:
        msg.canWriteByte(bus,node,2,data[0],data[1])
        print("Message sent on {}, data: {}".format(bus,msg))
    except Exception,e:
        print("Message NOT sent, Error: {}".format(e))

#define the function to send commands to all sensor controllers
def sendToAllControllers(msg = []):
    global board_start_num
    global num_of_board
    global cif_array
    global id_base
    for j in range (board_start_num, num_of_board+board_start_num): 
        if can_bustype == "esd":
            send_data_esd((id_base|j),msg,cif_array[j,0],cmsg_array[j,0])
        else:
            send_data((id_base|j),msg,cif_array[j,0])
    print("Sent {} to all {} controllers".format(msg,num_of_board))

#define the function to run on errors
def stopSystem(e = "", msg = type('',(),{})()):
    global conn
    global stop_threads
    stop_threads = True
    print("Program was stopped. ({})".format(e))
    sendToAllControllers([7,1]) #send stop command to all controllers
    try:
        print(msg.arbitration_id) #print the message Taxel ID, if this is not available, no message will be printed
        print(msg) #print the message object
        ar = [msg.arbitration_id] #make array for data with taxel ID as first element
        ar.append(msg.data) #append the data to keep bytes separate
        print(ar) #print the message data
    except Exception,e:
        print("No valid message supplied")
    try:
        csvfile.close()
    except Exception,e:
        print("Unable to close CSV file, Error: {}".format(e))
    try:
        conn.shutdown(socket.SHUT_RDWR)
        time.sleep(2)
        conn.close()
    except Exception,e:
        print("Unable to disable the server, Error: {}".format(e))
    exit() #end the program

############################# setup ######################################
#Generate CAN address
address_list = []
CAN_address = []
CAN_temp = []
#load correct addresslist
if num_sda == 4 and num_of_chip == 4:
    address_list = [0x100, 0x101, 0x102, 0x103,
                    0x110, 0x111, 0x112, 0x113,
                    0x120, 0x121, 0x122, 0x123,
                    0x130, 0x131, 0x132, 0x133
                    ]
elif num_sda == 4 and num_of_chip == 6:
    address_list = [0x130, 0x131, 0x132, 0x133, 0x134, 0x135,
                    0x120, 0x121, 0x122, 0x123, 0x124, 0x125,
                    0x110, 0x111, 0x112, 0x113, 0x114, 0x115,
                    0x100, 0x101, 0x102, 0x103, 0x104, 0x105
                    ]
else:
    address_list = [0x100, 0x101, 0x102, 0x103,
                    0x110, 0x111, 0x112, 0x113,
                    0x120, 0x121, 0x122, 0x123,
                    0x130, 0x131, 0x132, 0x133
                    ]
CAN_address.append(address_list)


for j in range(board_start_num, num_of_board+board_start_num): #setup the interfaces
    try:
        if can_bustype != "esd":
            try:
                cif_array[j,0] = can.interface.Bus(bustype=can_bustype, channel=can_channel, bitrate=1000000, ttyBaudrate=1000000)
            except Exception,e:
                if osname == "Windows":
                    print("Falling back to esd")
                    can_bustype = "esd"
                    import ntcan
                    for k in range(0, num_taxel):
                        cif_array[j,k] = ntcan.CIF(0,1)
                        cif_array[j,k].baudrate = 0
                        cif_array[j,k].canIdAdd(CAN_address[j-board_start_num][k])
                else:
                    stopSystem("Connection error (line: 204)")
        else:
            print("Using esd controller")
            for k in range(0, num_taxel):
                cif_array[j,k] = ntcan.CIF(0,1)
                cif_array[j,k].baudrate = 0
                cif_array[j,k].canIdAdd(CAN_address[j-board_start_num][k])
    except Exception,e:
        print("Error connecting to CAN: {}".format(e))
        exit()

for j in range(board_start_num, num_of_board+board_start_num): #setup the message handlers
    for k in range(0, num_taxel):
        if can_bustype == "esd":
            cmsg_array[j,k] = ntcan.CMSG()
        else:
            cmsg_array[j,k] = can.Message()

sendToAllControllers([7,0])#Let's start all controllers


print('Setup is completed')

############################# Baseline ###################################
print('Recording a baseline...')
print('Please wait')

mlx_buffer = {}
for j in range (board_start_num, num_of_board+board_start_num): #give default values to all buffer elements to avoid reference errors
	for k in range(0,num_taxel):
		key = {}
		for l in range(1,7):
			key[l] = 0x00
		mlx_buffer[j,k] = key

errcount = 0
for j in range(0,num_of_board):
    write_to = j
    del_num = board_start_num + j
    csvfile = open(path2 + 'LOG%s.csv' %del_num,'wb' ) #Create a new csv file
    filewrite = csv.writer(csvfile)
    time.sleep(1)
    start = int(round(time.time() * 1000))
    for i in range (-20,100):
        msg = {}
        try:
            for j in range (board_start_num, num_of_board+board_start_num):
                if can_bustype == "esd":
                    for k in range (0, num_taxel):
                        cmsg_array[j,k].canRead(cif_array[j,k])
                        mlx_buffer[j,k] = cmsg_array[j,k].data.c
                else:    
                    msg = cif_array[j,0].recv(1)
                    try:
                        for k in range(0,num_taxel):
                            try:
                                if msg == None:
                                    print("Try {} @ [{},{}] is unavailable".format(i,j,k))
                                    errcount += 1
                                    if errcount < 160:
                                        continue
                                    else:
                                        stopSystem("port unavailable")
                                elif msg.arbitration_id == address_list[k]:
                                    ##split into array of 8
                                    ar = []
                                    ar.extend(msg.data)
                                    mlx_buffer[j,k] = ar
                            except Exception,e:
                                print("MSG: {}".format(msg))
                                stopSystem("Error with msg in baseline: {}".format(e),msg)
                    except Exception,e:
                        stopSystem(e,msg)
        except Exception,e:
            stopSystem("Error in reading baseline. ({})".format(e),msg)
        
        
        #combine MSB | LSB
        x_axis = {}
        y_axis = {}
        z_axis = {}
        label = {}
        for j in range (board_start_num, num_of_board+board_start_num):
            label[j] = list()
            for k in range (0, num_taxel):
                try:
                    x_axis[j,k] = mlx_buffer[j,k][1] << 8 | mlx_buffer[j,k][2]
                    y_axis[j,k] = mlx_buffer[j,k][3] << 8 | mlx_buffer[j,k][4]
                    z_axis[j,k] = mlx_buffer[j,k][5] << 8 | mlx_buffer[j,k][6]
                    label[j].extend([x_axis[j,k], y_axis[j,k], z_axis[j,k]])
                except KeyError,e:
                    stopSystem("key error: {}".format(e))
                    print(mlx_buffer)
                
            time.sleep(0.002) #time required to save info
            if i>=0:#to eliminate zero data at the beginning, we will ignore few sets
                filewrite.writerow(label[j])
    
    end = int(round(time.time() * 1000))
    print("Read for 120 times took {}ms".format(end-start))
    csvfile.close()
print('Finished')


############################# TCP/IP #####################################

#setup the local server
TCP_IP = '127.0.0.1'
TCP_PORT = 5007
BUFFER_SIZE = 1
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
conn, addr = s.accept()
data_length = 6  # xyz * (MSB & LSB)
size = num_of_board * data_length * num_taxel
step = data_length * num_taxel
test_array = bytearray(size)
test_buffer = buffer(test_array,0, size)
print("Server ready")

############################# RUN ########################################

mlx_buffer = {}
key = {}
for l in range(0,8):
	key[l] = 0x00
for j in range (0, num_of_board+board_start_num):
    for k in range(0, num_taxel):
        mlx_buffer[j,k] = key

def canReaderESD(): #esd specific read thread
    global mlx_buffer
    global cif_array
    global num_of_board
    global board_start_num
    global stop_threads
    while(True):
        if(stop_threads):
            print("\nStopping Thread 1\n")
            break
        for j in range (board_start_num, num_of_board+board_start_num):
            for k in range (0, num_taxel):
                cmsg_array[j,k].canRead(cif_array[j,k])
                datastream = cmsg_array[j,k].data.c
                mlx_buffer[j-board_start_num,k] = datastream

def canReader():
    global mlx_buffer
    global cif_array
    global num_of_board
    global board_start_num
    global stop_threads
    while(True):
        if(stop_threads):
            print("\nStopping Thread 1\n")
            break
        for j in range (board_start_num, num_of_board+board_start_num):
            try:
                msg = cif_array[j,0].recv(1)
            except Exception,e:
                print("Problem on receive: {}".format(e))
                pass
            for _ in range(0,num_taxel):
                try:
                    arb = address_list.index(msg.arbitration_id)
                    datastream = memoryview(msg.data).tolist()
                    mlx_buffer[j-board_start_num,arb] = datastream
                except ValueError:
                    pass
                except Exception,e:
                    stopSystem("Error in thread-1 ({})".format(e))
                    break

#add worker
if can_bustype == "esd":
    t = threading.Thread(target=canReaderESD)
else:
    t = threading.Thread(target=canReader)
t.daemon = True
t.start()

try:
    while(True):
        #Put all MSB & LSB in a buffer
        for j in range (0, num_of_board):
            for k in range (0, num_taxel):
                try:
                    for s in range(0,6):
                        test_array[j*step + k*6 + s] = mlx_buffer[j,k][s+1]
                        if test_array[j*step + k*6 + s] < 250:
                            test_array[j*step + k*6 + s] += 2
                except Exception,e:
                    try:
                        print("Buffer for [{},{}]: {}".format(j,k,mlx_buffer[j,k]))
                    except Exception,er:
                        print("Error for accessing buffer at [{},{}]: {}".format(j,k,er))
                        print(mlx_buffer)
                    stopSystem("Error in main loop ({})".format(e))
        conn.sendall(test_array)
except KeyboardInterrupt:
    stopSystem("User cancellation")
except Exception,e:
    stopSystem(e)
