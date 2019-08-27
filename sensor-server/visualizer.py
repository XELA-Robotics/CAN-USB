#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as patches
from matplotlib.figure import Figure
from matplotlib import cm
from matplotlib import animation
import array
import socket  
import math   
import datetime
import threading
import time
#CSV reading
import csv
from collections import Counter
import matplotlib as mpl
mpl.rcParams['toolbar'] = 'None'

import ConfigParser #for INI parsing
import io #for INI parsing

#parse the INI file
with open("config.ini") as f:
    sample_config = f.read()
config = ConfigParser.RawConfigParser(allow_no_value=True)
config.readfp(io.BytesIO(sample_config))

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
if config.get('viz','max_offset'):
    maxval = int(config.get('viz','max_offset'))/10.0
else:
    maxval = 1.0
if config.get('viz','max_size'):
    maxsize = int(config.get('viz','max_size'))/10.0
else:
    maxsize = 5.0
num_taxels = num_sda * num_of_chip



#load display config based on sda and chip values
multiplier = {
    4: 2,
    6: 1.5,
    8: 1,
}
ofs = {
    4: 0,
    6: -0.3333,
    8: 0.5
}
ofs_x = ofs.get(num_of_chip,0)
ofs_y = ofs.get(num_sda,0)
mult_x = multiplier.get(num_of_chip,2)
mult_y = multiplier.get(num_sda,2)
print_lock = threading.Lock() #set lock for printing
def atomic_print(msg):
    print_lock.acquire()
    print msg
    print_lock.release()

def average_column (csv_filepath):
    column_totals = Counter()
    with open(csv_filepath,"rb") as f:
        reader = csv.reader(f)
        row_count = 0.0
        for row in reader:
            for column_idx, column_value in enumerate(row):
                try:
                    n = float(column_value)
                    column_totals[column_idx] += n
                except ValueError:
                    print "Error -- ({}) Column({}) could not be converted to float!".format(column_value, column_idx)                    
            row_count += 1.0   
    # row_count is now 1 too many so decrement it back down
    row_count -= 1.0
    # make sure column index keys are in order
    column_indexes = column_totals.keys()
    column_indexes.sort()

    # calculate per column averages using a list comprehension
    averages = [int(column_totals[idx]/row_count) for idx in column_indexes]
    return averages


fig = plt.figure(facecolor='black')
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)

figcfg = plt.gcf()
figcfg.canvas.set_window_title('Visualizer')
figcfg.canvas.facecolor = 'k'
ax = fig.add_subplot(111)
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)

line_color = 'xkcd:dark blue'
circle_color = 'xkcd:lime green'

#set background
rect = patches.Rectangle((0,0),10,10,facecolor='black')
ax.add_patch(rect)
#define grid
lines_x = []
lines_y = []
for i in range(0,num_of_chip):
    lines_x.append(plt.plot([(i+1)*mult_x+ofs_x,(i+1)*mult_x+ofs_x], [0,10], linewidth=1, color=line_color))
for i in range(0,num_sda):
    lines_y.append(plt.plot([0,10], [(i+1)*mult_y+ofs_y,(i+1)*mult_y+ofs_y], linewidth=1, color=line_color))
#define taxels
tst = []
positions = []
datastream = []
tres = []
for j in range(0,num_sda):
    for i in range(0,num_of_chip):
        pos_x = 10.0-((i+1)*mult_x+ofs_x)
        pos_y = 10.0-((j+1)*mult_y+ofs_y)
        tst.append(plt.Circle((pos_x,pos_y),0.01,fc=circle_color))
        positions.append([pos_x,pos_y])
        datastream.append([66,61,66,59,0,100])
        tres.append([16957, 16655, 38421])
        print("Object[{},{}]({}) at [{},{}] registered".format(i,j,(i*num_of_chip+j),(i+1)*mult_x+ofs_x,(j+1)*mult_y+ofs_y))

def init_treshold():
    global tres
    #tres = []
    tresholds = average_column("LOG1.csv")
    for i in range(0,len(datastream)):
        try:
            #tres.append([tresholds[i*3],tresholds[i*3+1],tresholds[i*3+2]])
            tres[i] = [tresholds[i*3],tresholds[i*3+1],tresholds[i*3+2]]
        except IndexError as e:
            print("[Tres] Error at index {}: {}".format(i,e))



def convert(list):
    return tuple(i for i in list) 

def init():
    global tst
    for i in range(0,len(tst)):
        ax.add_patch(tst[i])
    return convert(tst)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def calc(a,b,c=-1,d=-1):
    global tres,maxval,maxsize
    dbyte = a*256.0+b
    if d==2:
        if dbyte<tres[c][2]:
            dbyte = tres[c][2]
        valnew = translate(dbyte,tres[c][2],256*256,0.0,maxsize)
    elif d==1:
        if dbyte<tres[c][0]:
            valnew = translate(dbyte,0,tres[c][0],-maxval,0.0)*(-1)
        else:
            valnew = translate(dbyte,tres[c][0],256*256,0.0,maxval)*(-1)
    else:
        if dbyte<tres[c][1]:
            valnew = translate(dbyte,0,tres[c][1],-maxval,0.0)*(-1)
        else:
            valnew = translate(dbyte,tres[c][1],256*256,0.0,maxval)*(-1)
    #translate(sensor_value, 1, 512, 5, 10)
    return valnew+0.1

def realCalc(obj_id):
    global datastream,positions
    data = datastream[obj_id]
    val_x = calc(data[0],data[1],obj_id,0)
    val_y = calc(data[2],data[3],obj_id,1)
    pos = positions[obj_id]
    return float(calc(data[4],data[5],obj_id,2)),(float(pos[0]+val_x),float(pos[1]+val_y))

def setReady():
    global tst
    init_treshold()
    for i in range(0,len(tst)):
        tst[i].set_facecolor(circle_color)
    return convert(tst)

def rmReady():
    global tst
    for i in range(0,len(tst)):
        tst[i].set_facecolor('r')
    return convert(tst)

def animate(i):
    global datastream
    for i in range(0,len(tst)):
        tst[i].radius,tst[i].center = realCalc(i)
    return convert(tst)

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames = 360,
                               interval=1,
                               blit=True)

running = True
t2running = True
connected = False
ready = False
started = False

rmReady()

s = socket.socket() #define before loop
s.settimeout(0.5)    
port = 5007  

def dataManipulator():
    atomic_print("DM")
    global datastream,running,s,connected,ready,started,num_taxels
    started = True
    setReady()
    errcount = 0
    while(True):
        if running:
            pass
        else:
            atomic_print("Failure in Secondary thread")
            running = False
            connected = False
            started = False
            rmReady()
            s.close()
            break
        #data = memoryview(s.recv(6*16)).tolist()
        dataraw = s.recv(6*num_taxels)
        data = memoryview(dataraw).tolist()
        try:
            for i in range(0,len(datastream)):
                datastream[i] = [data[i*6],data[i*6+1],data[i*6+2],data[i*6+3],data[i*6+4],data[i*6+5]]
            ready = True
        except Exception,e:
            atomic_print("Error (T1): {}".format(e))
            if errcount > 1:
                running = False
                connected = False
                started = False
                rmReady()
                s.close()
                break
            else: 
                errcount += 1

def conThread():
    global connected,running,t2running,s,started
    while(True):
        #atomic_print("T2on: {}".format(t2running))
        if connected:
            try:
                if not started:
                    atomic_print("Start DM")
                    t = threading.Thread(target=dataManipulator)
                    t.daemon = True
                    t.start()
                else: 
                    pass
            except socket.error:
                atomic_print("Connection lost")
                connected = False
            except IndexError,e:
                atomic_print("Index failure: {}".format(e))
                running = False
                connected = False
                time.sleep(0.5)
                s.close()
            except Exception,e:
                atomic_print("Error: {}".format(e))
                running = False
                connected = False
                time.sleep(0.5)
                s.close()
                atomic_print("Good bye!")
                t2running = False
        else:
            try:            
                s.connect(('127.0.0.1', port)) 
                connected = True
                running = True
                print("Connected")
                time.sleep(0.2)
                init_treshold()
                time.sleep(0.1)
            except socket.error as e:
                atomic_print("Unable to connect. Retrying... {}".format(e))
                s.close()
                time.sleep(0.5)
                s = socket.socket()
                running = False
                connected = False
                time.sleep(2)
            except Exception,e:
                atomic_print("Error: {}".format(e))
                running = False
                connected = False
                time.sleep(0.5)
                s.close()
                atomic_print("Good bye!")
                t2running = False
    atomic_print("Ending connection Thread")



t2 = threading.Thread(target=conThread)
t2.daemon = True
t2.start()
plt.show()
print("Draw failed")
            
