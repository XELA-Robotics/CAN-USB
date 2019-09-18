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
import sys #for running several instances
import platform
import os
import Tkinter as tk
from matplotlib.widgets import Button

osname = platform.system()
fnamewpath = os.path.realpath(__file__)
fnamenpath = os.path.basename(fnamewpath)
if osname == "Windows":
    path2 = fnamewpath[0:(len(fnamewpath)-len(fnamenpath))].replace("\\","/")
    path2p = 'python "{}"'.format(fnamewpath.replace("\\","/"))
    path2i = "{}xela.ico".format(path2)
    path2ini = path2 + "config.ini"
else:
    path2 = fnamewpath[0:(len(fnamewpath)-len(fnamenpath))]
    path2p = "{}".format(fnamewpath)
    path2i = "{}xela.png".format(path2)
    path2ini = "/etc/xela/xServ.ini"
print("Open config at {}".format(path2ini))
with open(path2ini) as f:
    sample_config = f.read()
config = ConfigParser.RawConfigParser(allow_no_value=True)
config.readfp(io.BytesIO(sample_config))

def iniReader(handle,topic,element,defaultVal):
    try:
        return handle.get(topic,element)
    except ConfigParser.NoOptionError:
        return defaultVal
    except ConfigParser.NoSectionError:
        return defaultVal

#Get basic data from INI
with open(path2ini) as f:
    sample_config = f.read()
config = ConfigParser.RawConfigParser(allow_no_value=True)
config.readfp(io.BytesIO(sample_config))
num_sda = int(iniReader(config,'controller', 'num_sda', 4)) #number of sdas
num_of_chip = int(iniReader(config,'controller', 'num_chip', 4)) #number of chips per sda
num_of_board = int(iniReader(config,'controller', 'num_brd', 4)) #how many controllers
maxval = int(iniReader(config,'CAN', 'max_offset', '10'))/10.0 
maxsize = int(iniReader(config,'CAN', 'max_size', '50'))/10.0
sCD = [] #set up all controllers (properly as they might be very different)
for i in range(0,num_of_board):
    j = i+1
    sda = int(iniReader(config,'controller{}'.format(j), 'num_sda', num_sda))
    chp = int(iniReader(config,'controller{}'.format(j), 'num_chip', num_of_chip))
    sCD.append({
        'num_sda':sda,
        'num_of_chip':chp,
        'num_taxels':(sda*chp)
    })
limited = False
if len(sys.argv) > 1:
    if sys.argv[1] == "-l" or sys.argv[1] == "--limit":
        cur_viz = int(sys.argv[2])
        limited = True
    else:
        cur_viz = int(sys.argv[1])
else:
    cur_viz = 1
cur_brd = cur_viz - 1

print("sCD: {}".format(sCD[cur_brd]))
print("Viz: {}".format(cur_viz))

def VizStarter():
    global path2,cur_viz
    cmd = "{} {}".format(path2p,cur_viz+1)
    print("opening new viz '{}'".format(cmd))
    os.system(cmd)

if num_of_board > cur_viz and not limited:
    tr = threading.Thread(target=VizStarter)
    tr.daemon = True
    tr.start()

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
ofs_x = ofs.get(sCD[cur_brd]['num_of_chip'],0)
ofs_y = ofs.get(sCD[cur_brd]['num_sda'],0)
mult_x = multiplier.get(sCD[cur_brd]['num_of_chip'],2)
mult_y = multiplier.get(sCD[cur_brd]['num_sda'],2)
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
    if row_count < 1:
        row_count = 1
    # calculate per column averages using a list comprehension
    averages = [int(column_totals[idx]/row_count) for idx in column_indexes]
    return averages


fig = plt.figure(facecolor='black')
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)
#check if better icon exists
print("Check if {} exists".format("{}".format(path2i)))
if os.path.isfile("{}".format(path2i)):
    print("new icon found")
    thismanager = plt.get_current_fig_manager()
    img = tk.PhotoImage(file="{}".format(path2i))
    thismanager.window.tk.call('wm', 'iconphoto', thismanager.window._w, img)

figcfg = plt.gcf()
if num_of_board > 1:
    figcfg.canvas.set_window_title("Visualizer for sensor {}".format(cur_viz))
else:
    figcfg.canvas.set_window_title('Visualizer')
figcfg.canvas.facecolor = 'k'
ax = fig.add_subplot(111)
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
def on_click(event):
    setDefaults()
plt.connect('button_press_event', on_click)


line_color = 'xkcd:dark blue'
circle_color = 'xkcd:lime green'

#set background
rect = patches.Rectangle((0,0),10,10,facecolor='black')
ax.add_patch(rect)
#define grid
lines_x = []
lines_y = []
for i in range(0,sCD[cur_brd]['num_of_chip']):
    lines_x.append(plt.plot([(i+1)*mult_x+ofs_x,(i+1)*mult_x+ofs_x], [0,10], linewidth=1, color=line_color))
for i in range(0,sCD[cur_brd]['num_sda']):
    lines_y.append(plt.plot([0,10], [(i+1)*mult_y+ofs_y,(i+1)*mult_y+ofs_y], linewidth=1, color=line_color))

axcut = plt.axes([0.0, 0.0, 0.075, 0.02])
bcut = Button(axcut, 'Reset', color='red', hovercolor='green')

#define taxels
tst = []
positions = []
datastream = []
tres = []
baselinex = []
baseliney = []
for j in range(0,sCD[cur_brd]['num_sda']):
    for i in range(0,sCD[cur_brd]['num_of_chip']):
        pos_x = 10.0-((i+1)*mult_x+ofs_x)
        pos_y = 10.0-((j+1)*mult_y+ofs_y)
        tst.append(plt.Circle((pos_x,pos_y),0.01,fc=circle_color))
        positions.append([pos_x,pos_y])
        datastream.append([0,0,0,0,0,0])
        baselinex.append(0)
        baseliney.append(0)
        tres.append([32512, 32512, 0])
        print("Object[{},{}]({}) at [{},{}] registered".format(i,j,(i*num_of_chip+j),(i+1)*mult_x+ofs_x,(j+1)*mult_y+ofs_y))

def init_treshold():
    global tres,cur_viz
    #tres = []
    print("Opening LOG{}.csv".format(cur_viz))
    try:
        tresholds = average_column("LOG{}.csv".format(cur_viz))
    except Exception,e:
        print("Error (Threshold): {}: {}".format(type(e).__name__,e))
    for i in range(0,len(datastream)):
        try:
            #tres.append([tresholds[i*3],tresholds[i*3+1],tresholds[i*3+2]])
            tres[i] = [tresholds[i*3],tresholds[i*3+1],tresholds[i*3+2]]
        except IndexError as e:
            print("[Tres] Error at index {}: {}".format(i,e))

def setDefaults():
    global tres,cur_viz,baselinex,baseliney
    for i in range(0,len(datastream)):
        try:
            baselinex[i] = 0
            baseliney[i] = 0
            tres[i] = [0x7FFF,0x7FFF,0xFFFF] #x and y middle, z as max
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
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    if leftSpan < 0.1 or rightSpan < 0.1:
        return 0.0
    else:
        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)

def calc(a,b,c=-1,d=-1):
    global tres,maxval,maxsize,baselinex,baseliney
    dbyte = a*256.0+b
    if dbyte == 0:
        return 0.1
    if d==2: #z
        if dbyte<tres[c][2]:
            if dbyte > 0:
                tres[c][2] = dbyte
            else:
                return 0.1
        valnew = translate(dbyte,tres[c][2],256*256,0.0,maxsize)
        valnew += 0.04
    elif d==1: #x
        if baselinex[c] == 0:
            tres[c][0] = dbyte
            baselinex[c] = 1
        if dbyte<tres[c][0]:
            valnew = translate(dbyte,0,tres[c][0],-maxval,0.0)*(-1)
        else:
            valnew = translate(dbyte,tres[c][0],256*256,0.0,maxval)*(-1)
    else: #y
        if baseliney[c] == 0:
            tres[c][1] = dbyte
            baseliney[c] = 1
        if dbyte<tres[c][1]:
            valnew = translate(dbyte,0,tres[c][1],-maxval,0.0)*(-1)
        else:
            valnew = translate(dbyte,tres[c][1],256*256,0.0,maxval)*(-1)
    #translate(sensor_value, 1, 512, 5, 10)
    return valnew

def realCalc(obj_id):
    global datastream,positions
    data = datastream[obj_id]
    if data[0]+data[1]+data[2]+data[3]+data[4]+data[5] == 0:
        tst[obj_id].set_facecolor('r')
    else:
        tst[obj_id].set_facecolor(circle_color)
    val_x = calc(data[0],data[1],obj_id,1)
    val_y = calc(data[2],data[3],obj_id,0)
    pos = positions[obj_id]
    return float(calc(data[4],data[5],obj_id,2)),(float(pos[0]+val_x),float(pos[1]+val_y))

#def setReady():
#    global tst
#    init_treshold()
#    for i in range(0,len(tst)):
#        tst[i].set_facecolor(circle_color)
#    return convert(tst)

def rmReady():
    global tst,datastream
    for i in range(0,len(tst)):
        datastream[i] = [0,0,0,0,0,0]
#        tst[i].set_facecolor('r')
#    return convert(tst)

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
port = 5007+cur_brd  

def dataManipulator():
    atomic_print("DM")
    global datastream,running,s,connected,ready,started,sCD
    num_taxels = sCD[cur_brd]['num_taxels']
    setDefaults()
    started = True
#    setReady()
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
                atomic_print("(332)Error: {}: {}".format(type(e).__name__,e))
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
#                init_treshold()
                setDefaults()
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
                atomic_print("(357) Error: {}: {}".format(type(e).__name__,e))
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
            
