#Sensor Server
#by Matt
#release 1.1
#Support USB2CAN & PCAN

import can #main library handling the devices
import math #currently not required
import time #timing purposes
import getopt #currently not required
import sys #currently not required
import string #currently not required
import os
import thread #currently not required
import util #currently not required
import threading #currently not required
import types
import csv
import socket

conn = type('',(),{})() #define the empty server object in case there is failure before it is ready to be started
stop_threads = False #variable to stop all threads

id_base = 0x200 #Target ID of the MTB
cif_array = {} #can interface array
cmsg_array = {} #array for message handlers
num_of_board = 1 #how many controllers
board_start_num = 1 #first controller number
csvfile = type('',(),{})() #define the object for csvfile creation

num_sda = 4 #number of sdas
num_of_chip = 4 #number of chips per sda
num_taxel = num_sda * num_of_chip #total taxels per sensor

#define function to send data to controllers
def send_data(node,data,bus):
    msg = can.Message(arbitration_id=node,data=data,extended_id=False)
    try:
        bus.send(msg)
        print("Message sent on {}, data: {}".format(bus.channel_info,msg))
    except Exception,e:
        print("Message NOT sent, Error: {}".format(e))

#define the function to send commands to all sensor controllers
def sendToAllControllers(msg = []):
    global board_start_num
    global num_of_board
    global cif_array
    global id_base
    for j in range (board_start_num, num_of_board+board_start_num): 
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
        conn.close()
    except Exception,e:
        print("Unable to disable the server, Error: {}".format(e))
    exit() #end the program

############################# setup ######################################

for j in range(board_start_num, num_of_board+board_start_num): #setup the interfaces
    try:#start PCAN
        cif_array[j,0] = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
    except Exception:#if not possible
        try:#start USB2CAN
            cif_array[j,0] = can.interface.Bus(channel='ED000200', bustype='usb2can', bitrate=1000000)
        except Exception:#no CAN possible
            stopSystem("CAN not available")#we can't continue, so lets close the app

for j in range(board_start_num, num_of_board+board_start_num): #setup the message handlers
    for k in range(0, num_taxel):
        cmsg_array[j,k] = can.Message()

sendToAllControllers([7,0])#Let's start all controllers

#Generate CAN address
address_list = []
CAN_address = []
CAN_temp = []
address_list = [0x100, 0x101, 0x102, 0x103,
               0x110, 0x111, 0x112, 0x113,
                0x120, 0x121, 0x122, 0x123,
               0x130, 0x131, 0x132, 0x133]
CAN_address.append(address_list)
print('Setup is completed')

############################# Baseline ###################################
print('Recording a baseline...')
print('Please wait')

mlx_buffer = {}
for j in range (board_start_num, num_of_board+board_start_num): #give default values to all buffer elements to avoid reference errors
	for k in range(0,16):
		key = {}
		for l in range(1,7):
			key[l] = 0x00
		mlx_buffer[j,k] = key

for j in range(0,num_of_board):
    write_to = j
    del_num = board_start_num + j
    csvfile = open('LOG%s.csv' %del_num,'wb' ) #Create a new csv file
    filewrite = csv.writer(csvfile)
    time.sleep(1)
    for i in range (-20,100):
        try:
            for j in range (board_start_num, num_of_board+board_start_num):
                msg = cif_array[j,0].recv(1)
                try:
                    for k in range(0,num_taxel):
                        if msg.arbitration_id == address_list[k]:
                            ##split into array of 8
                            ar = []
                            ar.extend(msg.data)
                            mlx_buffer[j,k] = ar
                except Exception,e:
                    stopSystem(e,msg)
        except Exception,e:
            stopSystem(e,msg)
        

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
    csvfile.close()
print('Finished')


############################# TCP/IP #####################################

#setup the local server
TCP_IP = '127.0.0.1'
TCP_PORT = 5007
BUFFER_SIZE = 1
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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
for l in range(1,7):
	key[l] = 0x00
for j in range (0, num_of_board+board_start_num):
	for k in range(0,num_taxel):
		mlx_buffer[j,k] = key
err_count = 0

def canReader():
    global err_count
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
            msg = cif_array[j,0].recv(1)
            for k in range(0,num_taxel):
                arb = address_list.index(msg.arbitration_id)
                mlx_buffer[j-board_start_num,arb] = memoryview(msg.data).tolist()

#add worker
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
                    stopSystem("Error in main loop ({})".format(e))
        conn.sendall(test_array)
except KeyboardInterrupt:
    stopSystem("User cancellation")
except Exception,e:
    stopSystem(e)
