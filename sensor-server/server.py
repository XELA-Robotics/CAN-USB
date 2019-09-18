#!/usr/bin/env python

#Sensor Server
#by Matt
#release 1.2T
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

from inspect import currentframe, getframeinfo #for debug
import copy
import subprocess #for baselining outside of main code
#execute ./baseliner.py socketcan can0 513 1 4x4 old 16
#format  program.py bus_type channel MTB_id CSV_no size(taxel layout) version(new or old) taxelcount

osname = platform.system()
fnamewpath = os.path.realpath(__file__)
fnamenpath = os.path.basename(fnamewpath)
if osname == "Windows":
    path2 = fnamewpath[0:(len(fnamewpath)-len(fnamenpath))].replace("\\","/")
    path2p = 'python "{}"'.format(path2)
    path2ini = path2 + "config.ini"
else:
    #in Linux the config should be always in /etc/xela folder
    path2 = "/etc/xela/"
    path2p = "python {}".format(fnamewpath[0:(len(fnamewpath)-len(fnamenpath))])
    path2ini = "/etc/xela/xServ.ini"
#check if ini file exists
if os.path.isfile(path2ini):
    pass
else:
    #run configurator!!!
    if os.path.isfile("{}configurator.pyc".format(path2p)):
        cmd = "{}configurator.pyc".format(path2p)
    else:
        cmd = "{}configurator.py".format(path2p)
    print("Running: '{}'".format(cmd))
    os.system(cmd)
print("Open config at {}".format(path2ini))
with open(path2ini) as f:
    sample_config = f.read()
config = ConfigParser.RawConfigParser(allow_no_value=True)
config.readfp(io.BytesIO(sample_config))

#Function to read all specific values from config.ini
def iniReader(handle,topic,element,defaultVal):
    try:
        return handle.get(topic,element)
    except ConfigParser.NoOptionError:
        return defaultVal
    except ConfigParser.NoSectionError:
        return defaultVal

#Get basic data from INI
id_base = int(iniReader(config,'controller', 'id_base', 0x201)) #Target ID of the MTB
num_sda = int(iniReader(config,'controller', 'num_sda', 4)) #number of sdas
num_of_chip = int(iniReader(config,'controller', 'num_chip', 4)) #number of chips per sda
num_of_board = int(iniReader(config,'controller', 'num_brd', 4)) #how many controllers
can_bustype = iniReader(config,'CAN', 'bustype', '-1') #CAN Bus type (socketcan or pcan or slcan)
can_channel = iniReader(config,'CAN', 'channel', '-1') #CAN port (can0 or COM6 or /dev/ttyUSB0)
sens_type = int(iniReader(config,'controller','stype',0))

sensorConfigData = [] #set up all controllers (properly as they might be very different)
for i in range(1,num_of_board+1):
    ctrl_id = (id_base+i-1)
    sensorConfigData.append({
        'num_sda':int(iniReader(config,'controller{}'.format(i), 'num_sda', num_sda)),
        'num_of_chip':int(iniReader(config,'controller{}'.format(i), 'num_chip', num_of_chip)),
        'sens_type':int(iniReader(config,'controller{}'.format(i), 'stype', sens_type)),
        'ctrl_id':int(iniReader(config,'controller{}'.format(i), 'ctrl_id', ctrl_id))
    })

print(sensorConfigData)

if can_bustype == '-1' or can_channel == '-1': #if channel or bus is not defined, we can't run the server
    print "Config not available"
    exit()
if can_bustype == 'pcan' and num_of_board > 1:
    print("pcan currently only supports one MTB")
    #exit()

msgs = {
    'Windows-slcan':"Please be aware that SLCAN is very slow in Windows",
    'Windows-pcan':"PCAN is fully supported in Windows, please make sure you have PCANBasic.dll in right system folder",
    'Windows-serial':"Please be aware that Serial interface in Windows is extremely slow and lossy",
    'Windows-esd':"esd CAN-USB/2 is supported in Windows with separate thread",
    'Windows-socketcan':"socketcan is not supported in Windows",
    'Windows-other':"Windows only supports PCAN properly at this time",
    'Linux-socketcan':"Let's connect to CAN",
    'Linux-pcan':"Let's connect to CAN",
    'Linux-slcan':"SLCAN can be slow",
    'Linux-serial':"Serial can be slow",
    'Linux-other':"Unsupported Bus"
}

def msghdlr(osname,canbus):
    osc = osname + "-" + canbus
    if osname != 'Windows' and osname != 'Linux':
        print("Unsupported OS")
        exit()
    if osc == 'Windows-esd':
        import ntcan
    try:
        print(msgs[osc])
    except KeyError:
        osc = osname + "-other"
        try:
            print(msgs[osc])
        except KeyError,e:
            print("Data for OS not found. Error: {}".format(e))
            exit()
    if osc == 'Windows-other' or osc == 'Linux-other':
        exit()
    
msghdlr(osname,can_bustype)


servers = []
test_array = []
test_buffer = []
for i in range(0,num_of_board):
    servers.append({'running':False,'s':socket.socket(socket.AF_INET, socket.SOCK_STREAM),'er':False,'conn':type('',(),{})()})
    test_array.append([])
    test_buffer.append([])
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
            send_data_esd(sensorConfigData[j-1]['ctrl_id'],msg,cif_array[j,0],cmsg_array[j,0])
        else:
            send_data(sensorConfigData[j-1]['ctrl_id'],msg,can_bus)
    if num_of_board == 1:
        stxt = ''
    else:
        stxt = 's'
    print("Sent {} to {} controller{}".format(msg,num_of_board,stxt))

def sendToController(ctrl = 0,msg = []):
    global cif_array
    global id_base
    if can_bustype == "esd":
        send_data_esd(sensorConfigData[ctrl]['ctrl_id'],msg,cif_array[ctrl+1,0],cmsg_array[ctrl+1,0])
    else:
        send_data(sensorConfigData[ctrl]['ctrl_id'],msg,can_bus)
    print("Sent {} to controller{}".format(msg,ctrl+1))

#define the function to run on errors
def stopSystem(e = "", msg = type('',(),{})()):
    global servers
    global stop_threads
    global num_of_board
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
        for i in range(0,num_of_board):
            servers[i]["conn"].shutdown(socket.SHUT_RDWR)
            time.sleep(1)
            servers[i]["conn"].close()
            print("Server {} closed".format(i))
    except Exception,e:
        print("Unable to disable the server {}, Error: {}".format(i,e))
    exit() #end the program

############################# setup ######################################
#Generate CAN address
CAN_address = []
CAN_temp = []

#load config for addresses
for i in range(0,num_of_board):
    address_list = []
    adrini = "{}{}x{}.ini".format(path2,sensorConfigData[i]['num_sda'],sensorConfigData[i]['num_of_chip'])
    if sensorConfigData[i]['sens_type'] == 0:
        stype = "{}x{}-old".format(sensorConfigData[i]['num_sda'],sensorConfigData[i]['num_of_chip'])
    else:
        stype = "{}x{}-new".format(sensorConfigData[i]['num_sda'],sensorConfigData[i]['num_of_chip'])
    if os.path.exists(adrini):
        if os.path.isfile(adrini):
            #load and read config
            with open(adrini) as f:
                sdata = f.read()
            sini = ConfigParser.RawConfigParser(allow_no_value=True)
            sini.readfp(io.BytesIO(sdata))
            for i in range(1,(sensorConfigData[i]['num_sda']*sensorConfigData[i]['num_of_chip'])+1):
                address_list.append(int(iniReader(sini,stype,'s{}'.format(i),0)))

    print(address_list)
    CAN_address.append(address_list)

print(CAN_address)
if osname == "Windows" and can_bustype == "socketcan":
    can_bustype = "esd"
print("Connecting to {} via {}".format(can_bustype,can_channel))

#for i in range(0,num_of_board):
#    if sensorConfigData[i]['sens_type'] == 1:
#        version = "new"
#        print("sCD[{}] = {} => new".format(i,sensorConfigData[i]['sens_type']))
#    else:
#        version = "old"
#        print("sCD[{}] = {} => old".format(i,sensorConfigData[i]['sens_type']))
#    cmd = "{}baseliner.pyc {} {} {} {} {} {} {}".format(path2p,can_bustype,can_channel,sensorConfigData[i]['ctrl_id'],i+1,"{}x{}".format(sensorConfigData[i]['num_sda'],sensorConfigData[i]['num_of_chip']),version,sensorConfigData[i]['num_sda']*sensorConfigData[i]['num_of_chip'])
#    print("Running: '{}'".format(cmd))
#    os.system(cmd)
try:
    if can_bustype != "esd":
        can_bus = can.interface.Bus(bustype=can_bustype, channel=can_channel, bitrate=1000000, ttyBaudrate=1000000)
    else:
        print("Using esd controller")
        import ntcan
        for j in range(board_start_num, num_of_board+board_start_num):
            for k in range(0, num_taxel):
                cif_array[j,k] = ntcan.CIF(0,1)
                cif_array[j,k].baudrate = 0
                cif_array[j,k].canIdAdd(CAN_address[j-board_start_num][k])
except Exception,e:
    print("Error connecting to CAN: {}:{}".format(type(e).__name__,e))
    exit()
    

for j in range(board_start_num, num_of_board+board_start_num): #setup the message handlers
    for k in range(0, num_taxel):
        if can_bustype == "esd":
            cmsg_array[j,k] = ntcan.CMSG()
        else:
            cmsg_array[j,k] = can.Message()

#sendToAllControllers([7,0])#Let's start all controllers
time.sleep(1)


#run baseliner script!!!
#execute ./baseliner.py socketcan can0 513 1 4x4 old 16
#format  program.py bus_type channel MTB_id CSV_no size(taxel layout) version(new or old) taxelcount

#sensorConfigData[j-1]['num_sda']*sensorConfigData[j-1]['num_of_chip']
mlx_buffer = {}
buf = {}
key = {}
for l in range(1,7):
    key[l] = 0x77
for j in range (board_start_num, num_of_board+board_start_num): #give default values to all buffer elements to avoid reference errors
    num_taxel = (sensorConfigData[j-1]['num_sda']*sensorConfigData[j-1]['num_of_chip'])
    for k in range(0,num_taxel):
		mlx_buffer[j,k] = key
for k in range(0,num_taxel):
	buf[k] = key

print("Baseline finished")
sendToAllControllers([7,0])
time.sleep(1)
#exit()
############################# TCP/IP #####################################
def server(server_no = 0):
    #setup the local server
    global sensorConfigData
    global stop_threads
    global servers
    global test_array
    global test_buffer
    num_taxel = (sensorConfigData[server_no]['num_sda']*sensorConfigData[server_no]['num_of_chip'])
    TCP_IP = '127.0.0.1'
    TCP_PORT = 5007 + server_no
    servers[server_no]["s"].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    servers[server_no]["s"].bind((TCP_IP, TCP_PORT))
    data_length = 6  # xyz * (MSB & LSB)
    size = data_length * num_taxel
    servers[server_no]["step"] = data_length * num_taxel
    test_array[server_no] = bytearray(size)
    test_buffer[server_no] = buffer(test_array[server_no],0, size)
    while True:
        if(stop_threads):
            print("\nStopping Server Thread {}\n".format(server_no))
            servers[server_no]["running"] = False
            break
        try:
            servers[server_no]["s"].settimeout(0.2) # timeout for listening
            servers[server_no]["s"].listen(1) 
            servers[server_no]["conn"], (ip, port) = servers[server_no]["s"].accept()
        except socket.timeout:
            pass
        except Exception,e:
            print("Error with server {}: {}".format(server_no,e))
            servers[server_no]["running"] = False
            break
        else:
            servers[server_no]["running"] = True
            print("Server {} ready and connected to {}:{}".format(server_no,ip,port))
            while servers[server_no]["running"]:
                pass


############################# RUN ########################################
for i in range(0,num_of_board):
    print("Starting Server {}".format(i+1))
    ser = threading.Thread(target=server, args=(i,))
    ser.daemon = True
    ser.start()

while not servers[0]["running"]:
    pass
s = servers[0]["s"]
conn = servers[0]["conn"]
step = servers[0]["step"]

print("Server 0 in use")
num_taxel = 16
key = {}
for l in range(0,7):
	key[l] = 0x00
for j in range (0, num_of_board+board_start_num):
    for k in range(0, num_taxel):
        mlx_buffer[j,k] = key

def bufSetter(arb,data = []):
    global mlx_buffer
    global num_of_board
    global sensorConfigData
    global CAN_address
    for i in range(0,num_of_board):
        try:
            ar = CAN_address[i].index(arb)
        except ValueError:
            pass
        else:
            try:
                ax = data[0]+data[1]
                ay = data[2]+data[3]
                az = data[4]+data[5]
                if ax == 0 or ay == 0 or az == 0:
                    print("Data unavailable for {}: {},{},{} of {}".format(arb,ax,ay,az,data))
                else:
                    #print("Data available for {}: {},{},{} of {}".format(arb,ax,ay,az,data))
                    mlx_buffer[i,ar] = data
            except Exception:
                pass

def inArray(value,array):
    try:
        a = array.index(value)
    except ValueError:
        return False
    else:
        return True

def canReaderESD(): #esd specific read thread
    global mlx_buffer
    global cif_array
    global num_of_board
    global board_start_num
    global stop_threads
    global sensorConfigData
    bad_sens = []
    while(True):
        if(stop_threads):
            print("\nStopping CAN Thread\n")
            break
        for j in range (board_start_num, num_of_board+board_start_num):
            num_taxel = (sensorConfigData[j-1]['num_sda']*sensorConfigData[j-1]['num_of_chip'])
            for k in range (0, num_taxel):
                if inArray("{}-{}".format(j,k),bad_sens):
                    pass
                else:
                    try:
                        cmsg_array[j,k].canRead(cif_array[j,k])
                    except Exception,e:
                        print("[CRESD] Error on receive: {}: {}".format(type(e).__name__,e))
                        bad_sens.append("{}-{}".format(j,k))
                        datastream = [0,0,0,0,0,0,0,0]
                    else:
                        datastream = cmsg_array[j,k].data.c
                        mlx_buffer[j-board_start_num,k] = datastream

def canReader():
    global mlx_buffer
    global cif_array
    global num_of_board
    global board_start_num
    global stop_threads
    global sensorConfigData
    while(True):
        if(stop_threads):
            print("\nStopping CAN Thread\n")
            break
        try:
            msg = can_bus.recv(1)
        except Exception,e:
            print("Problem on receive: {}".format(e))
            pass
        else:
            bufSetter(msg.arbitration_id,memoryview(msg.data).tolist())

#add worker
if can_bustype == "esd":
    t = threading.Thread(target=canReaderESD)
else:
    t = threading.Thread(target=canReader)
t.daemon = True
t.start()

#buffer threads
def bufThread(server_no):
    while not servers[server_no]["running"]:
        pass
    while True:
        try:
            while(True):
                mbuf = copy.copy(mlx_buffer)
                #Put all MSB & LSB in a buffer
                num_taxel = (sensorConfigData[server_no]['num_sda']*sensorConfigData[server_no]['num_of_chip'])
                for k in range (0, num_taxel):
                    try:
                        for s in range(0,6):
                            test_array[server_no][k*6 + s] = mbuf[server_no,k][s+1]
#                            if test_array[server_no][k*6 + s] < 250:
#                                test_array[server_no][k*6 + s] += 2
                    except Exception,e:
                        try:
                            print("Buffer for [{},{}]: {}".format(j,k,mbuf[server_no,k]))
                        except Exception,er:
                            print("Error for accessing buffer at [{},{}]: {}".format(server_no,k,er))
                            print(mbuf)
                        stopSystem("Error in Buf Thread {} at {} ({}|{})".format(server_no,k,type(e).__name__,e))
                #print("{} | {}".format(test_array[server_no].count([0,0,0,0,0,0]),test_array[server_no]))
                #print(memoryview(test_array[server_no]).tolist())
                servers[server_no]["conn"].sendall(test_array[server_no])
        except KeyboardInterrupt:
            stopSystem("User cancellation ({})".format(server_no))
        except socket.error,e:
            print("Server {}: Connection lost: {}".format(server_no,e))
            servers[server_no]["running"] = False
            while not servers[server_no]["running"]:
                pass
            pass
        except Exception,e:
            stopSystem("SYS: {}".format(e))


for i in range(0,num_of_board):
    t = threading.Thread(target=bufThread, args=(i,))
    t.daemon = True
    t.start()

while True:
    try:
        time.sleep(0.01)
#        print("")
#        for i in range (0,16):
#            print(mlx_buffer[0,i])
    except KeyboardInterrupt:
        stopSystem("User cancellation (MAIN)")
    except Exception,e:
        stopSystem("SYS: {}".format(e))
