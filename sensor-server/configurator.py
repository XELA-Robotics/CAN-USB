#!/usr/bin/env python
 
import Tkinter as tk
import time
import easygui
import os
import platform

osname = platform.system()
fnamewpath = os.path.realpath(__file__)
fnamenpath = os.path.basename(fnamewpath)
if osname == "Windows":
    path2 = fnamewpath[0:(len(fnamewpath)-len(fnamenpath))].replace("\\","/")
    path2p = 'python "{}"'.format(path2)
    path2ini = path2 + "config.ini"
    path2i = "{}xela.png".format(path2)
else:
    #in Linux the config should be always in /etc/xela folder
    path2 = "/etc/xela/"
    path2e = fnamewpath[0:(len(fnamewpath)-len(fnamenpath))]
    path2p = "python {}".format(path2e)
    path2ini = "/etc/xela/xServ.ini"
    path2i = "{}xela.png".format(path2e)

but = []
answered = False
lastanswer = ""
configFileList = []

#class to replace text
def changeTo(elem,val):
    elem.config(state=tk.NORMAL)
    elem.delete("1.0",tk.END)
    appendTo(elem,val)
    elem.config(state=tk.DISABLED)

def changeLabel(label,val):
    label['text'] = val

#class to append text
def appendTo(elem,val):
    elem.insert(tk.END,val)

#class to update system
def updateView():
    #message update here
    changeTo(results,printList(configFileList))
    window.update()

#define possible bus types
bustype = [
    "socketcan",
    "esd",
    "pcan",
    "slcan"
    ]

#define possible ports
channel = [
    "can0",
    "PCAN_USBBUS1",
    "slcan0"
]
def showOptions(elemList,title = ""):
    global but, answered
    answered = False
    changeLabel(question,title)
    for i in range(0, len(but)):
        but[i].grid_forget()
    try:
        but.clear()
        but = []
    except Exception:
        del but[:]
        but = []    
    for i in range(0, len(elemList)):
        but.append(tk.Button(text=elemList[i], command = lambda val=elemList[i]: answer(val)))
        but[i].grid(row=i+1, column=0)
    while not answered:
        updateView()
def printList(array):
    data = ""
    for i in range(len(array)):
        if i>0:
            data = "{}\n".format(data)
        data = "{}{}".format(data,array[i])
    return data
def answer(val):
    global answered,lastanswer
    lastanswer = val
    answered = True
window = tk.Tk()
window.title("Xela Configurator")
window.geometry("900x500")
#check if better icon exists
#print("Check if {} exists".format("{}xela.ico".format(path2i)))
#if os.path.isfile("{}xela.ico".format(path2i)):
#    print("new icon found")
#    window.iconbitmap("{}xela.ico".format(path2i))
#check if better icon exists
print("Check if {} exists".format("{}".format(path2i)))
if os.path.isfile("{}".format(path2i)):
    print("new icon found")
#    thismanager = plt.get_current_fig_manager()
    img = tk.PhotoImage(file="{}".format(path2i))
#    thismanager.window.tk.call('wm', 'iconphoto', thismanager.window._w, img)
#    window.iconbitmap(default="{}".format(path2i))
    window.tk.call('wm', 'iconphoto', window._w, img)

window.resizable(width=tk.FALSE, height=tk.FALSE)
window.columnconfigure(1,weight=1)
question = tk.Label(window)
question.grid(row=0,column=0)
results = tk.Text(window,state=tk.DISABLED)
results.grid(row = 0, column = 1, rowspan = 5, sticky = "ne")
configFileList.append("[CAN]")
showOptions(bustype,"What bus are you using?")
configFileList.append("bustype = {}".format(lastanswer))
showOptions(channel,"What channel are you using?")
configFileList.append("channel = {}".format(lastanswer))
configFileList.append("[controller]")
##ask how many boards
isit = easygui.ynbox("Is there only 1 sensor on the channel?")
if isit:
    num_brd = 1
else:
    num_brd = easygui.enterbox("How many sensors on channel?") #tk.simpledialog.askstring(title="Config",prompt="How many boards on channel?")
configFileList.append("num_brd = {}".format(int(num_brd)))
updateView()
##ask default board sda
if num_brd > 1:
    onfirst = " 1st"
else:
    onfirst = ""
isit = easygui.ynbox("Are there 4 rows on the{} sensor?".format(onfirst))
if isit:
    num_sda = 4
else:
    num_sda = easygui.enterbox("How many rows per{} sensor?".format(onfirst))
configFileList.append("num_sda = {}".format(int(num_sda)))
updateView()
##ask default board chips
isit = easygui.ynbox("Are there 4 columns on the{} sensor?".format(onfirst))
if isit:
    num_chip = 4
else:
    num_chip = easygui.enterbox("How many columns per{} sensor?".format(onfirst))
configFileList.append("num_chip = {}".format(int(num_chip)))
updateView()
##ask default board id_base
isit = easygui.ynbox("Is the id of the{} sensor 0x201 (513)?".format(onfirst))
if isit:
    id_base = 513
else:
    id_base = easygui.enterbox("What is the starting id of the{} sensor (in decimal)".format(onfirst))
configFileList.append("id_base = {}".format(int(id_base)))
updateView()
##ask default board type
newold = easygui.ynbox("Is the{} board older type?".format(onfirst))
if newold:
    configFileList.append("stype = 0")
else:
    configFileList.append("stype = 1")
updateView()
#set up the Visualizer details
configFileList.append("[viz]")
configFileList.append("max_offset = 40")
configFileList.append("max_size = 50")
#go through all boards if more than 1
if int(num_brd) > 1:
    for i in range(2,int(num_brd)+1):
        is_not_same = easygui.ynbox("Is sensor {} different from 1st?".format(i))
        if is_not_same:
            configFileList.append("[controller{}]".format(int(i)))
            isit = easygui.ynbox("Does sensor {} have {} rows?".format(i,num_sda))
            if not isit:
                n_sda = easygui.enterbox("How many rows per sensor?")
                configFileList.append("num_sda = {}".format(int(n_sda)))
            updateView()
            ##ask board chips
            isit = easygui.ynbox("Does sensor {} have {} columns?".format(i,num_chip))
            if not isit:
                n_chip = easygui.enterbox("How many columns per sensor?")
                configFileList.append("num_chip = {}".format(int(n_chip)))
            updateView()
            ##ask board id_base
            isit = easygui.ynbox("Does sensor {} have id {} ({})?".format(i,hex(int(id_base)+i-1),int(id_base)+i-1))
            if not isit:
                i_base = easygui.enterbox("What is the starting id of the sensor number {} (in decimal)".format(i))
                configFileList.append("id_base = {}".format(int(i_base)))
            updateView()
            if newold:
                btstr = "an older"
                btold = True
            else:
                btstr = "a newer"
                btold = False
            isit = easygui.ynbox("Is sensor {} also {} type?".format(i,btstr))
            if not isit:
                if btold:
                    configFileList.append("stype = 1")
                else:
                    configFileList.append("stype = 0")
            updateView()
        else:
            print("Sensor {} is same".format(i))

print("Data:\n{}".format(printList(configFileList)))
updateView()
#write to ini file
inifile = open(path2ini,"w+")
for i in range(len(configFileList)):
    inifile.writelines("{}\n".format(configFileList[i]))
inifile.close()
exit()