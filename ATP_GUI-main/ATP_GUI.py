import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import collections


import serial
import serial.tools.list_ports
from serial import SerialException


# start collections with zeros
cpu = collections.deque(np.zeros(10))
ram = collections.deque(np.zeros(10))
xAccel= collections.deque(np.zeros(10))
yAccel= collections.deque(np.zeros(10))
zAccel= collections.deque(np.zeros(10))
alti=  collections.deque(np.zeros(10))

#number of reading coming in per line of serial
dataCount=24

#Column number which has the desired data in base-zero
xCol=5 #x-accel
yCol=6
zCol=7
altCol=10 #altitude is the 13th value in the data stream
latCol=11
longCol=12
satCol=13

#set graph y-axis limits, any data values outside the range will show a line going off the graph
#y-axis limits for x,y,z acceleration Graphs
accelMin=-200 
accelMax=200

#y-axis limits for Altitude Graph
altiMin=-100
altiMax=12000

#These variables control whether GUI will read from a data stream coming through a serial line, or a file
# (READFROMSERIAL + !READFROMFILE ) = read from Serial
# (READFROMSERIAL + READFROMFILE ) = read from Serial
# (!READFROMSERIAL + READFROMFILE ) = read from File
# (!READFROMSERIAL + !READFROMFILE ) = read Nothing, GUI does nothing

READFROMSERIAL=True
READFROMFILE=False



# Returns list of all accessible serial ports
def getPorts():
    portData = serial.tools.list_ports.comports()
    return portData

# Returns COM port of Arduino if detected by computer. User for switchbox
def findArduino(portsFound):
    numConnections = len(portsFound)
    for i in range(0, numConnections):
        if ('Uno' in str(portsFound[i]) or 'Nano' in str(portsFound[i]) or 'CH340' in str(portsFound[i])):
            return str(portsFound[i])

        # teensy 3.6
        if ('USB Serial Device' in str(portsFound[i])):
            return str(portsFound[i])
    return "None"


def conv(str):
    return str[2:len(str) - 5]


filename = open("TELEM_Data2.CSV",'r')


def my_function(i):
    if(not READFROMFILE and not READFROMSERIAL):
        return
    #reads from serial if variable is set to True
    if(READFROMSERIAL):
        try:
            strSerial = conv(str(arduinoSwitchbox.readline()))
            strSerial=strSerial.split(",")
        except SerialException:
            strSerial = ''
        print(strSerial)

        vals=strSerial

    #grab from file for testing, file should be in same directory
    #will only use file if READFROMFILE is True and READFROMSERIAL is False
    if(READFROMFILE and not READFROMSERIAL):
        data=""
        data= filename.readline()
        data=data.strip()
        vals = data.split(",")   
        for index, val in enumerate(vals):
            try:
                vals[index]=float(val)
            except ValueError:
                continue

    #if data line has less than the expected number of columns, add extra values until the dataCount is reached

    if(len(vals)<dataCount):
        vals.extend(["-"]*(dataCount-len(vals)))

 


    #pop from left of queue and append to its right

    #read in z-Acceleration if possible
    try:
        tempz=float(vals[zCol])
        zAccel.popleft()
        zAccel.append(tempz)
    except ValueError:
        tempz="No Reading"

    #read in y-Acceleration if possible
    try:
        tempy=float(vals[yCol])
        yAccel.popleft()
        yAccel.append(tempy)
    except ValueError:
        tempy="No Reading"
    
    #read in X-Acceleration if possible
    try:
        tempx=float(vals[xCol])
        xAccel.popleft()
        xAccel.append(tempx)
    except ValueError:   
        tempx="No Reading"
    #read in altitude if possible
    try:
        temp=float(vals[altCol])
        alti.popleft()
        alti.append(temp)
    except ValueError:
        temp=0

    #read in longitude, latitiude and number of satellites connected to GPS
    tempLong=vals[longCol]
    tempLat=vals[latCol]
    tempSat=vals[satCol]
   
    # clear axis
    ax1.cla()
    ax2.cla()
    ax3.cla()
    ax4.cla()
    #clear Text
    for txt in fig.texts:
        txt.set_visible(False)

    #Print Longitiude, Latitiude, Number of Satelites
    # https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.gcf.html

    plt.gcf().text(0.7, 0.5, "Longitude: ", fontsize=14)
    plt.gcf().text(0.8, 0.5, "{}".format(tempLong), fontsize=14)

    plt.gcf().text(0.7, 0.4, "Latititude: ", fontsize=14)
    plt.gcf().text(0.8, 0.4, "{}".format(tempLat), fontsize=14)

    plt.gcf().text(0.7, 0.3, "Satelites: ", fontsize=14)
    plt.gcf().text(0.8, 0.3, "{}".format(tempSat), fontsize=14)

    plt.gcf().text(0.52, 0.5, "X: ", fontsize=14)
    plt.gcf().text(0.54, 0.5, "{} m/s".format(tempx), fontsize=14)

    plt.gcf().text(0.52, 0.4, "Y: ", fontsize=14)
    plt.gcf().text(0.54, 0.4, "{} m/s".format(tempy), fontsize=14)

    plt.gcf().text(0.52, 0.3, "Z: ", fontsize=14)
    plt.gcf().text(0.54, 0.3, "{} m/s".format(tempz), fontsize=14)

   # plot 4- top right(Altitude)
    ax4.plot(alti)
    ax4.scatter(len(alti)-1, alti[-1])
    ax4.text(len(alti)-1, alti[-1]+2, "{} ft".format(alti[-1]))
    ax4.set_ylim(altiMin,altiMax)
    ax4.title.set_text("Altitude(ft.)")


   # plot 3 - bottom left(Z-Acceration) 
    ax3.plot(zAccel)
    ax3.scatter(len(zAccel)-1, zAccel[-1])
    ax3.text(len(zAccel)-1, zAccel[-1], "{}".format(zAccel[-1]))
    ax3.set_ylim(accelMin,accelMax)
    ax3.set_xticks([])
    ax3.title.set_text("Z-Acceleration")
    ax3.set_ylabel("m/s")



    # plot 2  - middle left(Y-Acceration)
    ax2.plot(yAccel)
    ax2.scatter(len(yAccel)-1, yAccel[-1])
    ax2.text(len(yAccel)-1, yAccel[-1], "{}".format(yAccel[-1]))
    ax2.set_ylim(accelMin,accelMax)
    ax2.set_xticks([])
    ax2.title.set_text("Y-Acceleration")
    ax2.set_ylabel("m/s")


    # plot 1 - top left(X-Acceration)
    ax1.plot(xAccel)   
    ax1.scatter(len(xAccel)-1, xAccel[-1])
    ax1.text(len(xAccel)-1, xAccel[-1], "{}".format(xAccel[-1]))
    ax1.set_ylim(accelMin,accelMax)
    ax1.set_xticks([])
    ax1.title.set_text("X-Acceleration")
    ax1.set_ylabel("m/s")




# define and adjust figure
# https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.figure.html
fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
fig.suptitle("Avionics Test Payload(ATP) GUI")

# https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.subplot.html
#(figure number of row, figure number of col, index)
ax1 = plt.subplot(3,2,1)
ax2 = plt.subplot(3,2,3)

ax1.set_facecolor('#DEDEDE')
ax2.set_facecolor('#DEDEDE')

ax3 = plt.subplot(3,2,5)
ax3.set_facecolor('#DEDEDE')

ax4 = plt.subplot(3,2,2)
ax4.set_facecolor('#DEDEDE')



# GET ARDUINO STATUS / Update on GUI connection label
status = findArduino(getPorts())
arduinoSwitchbox = serial.Serial()
if (not (status == "None")):
    arduinoSwitchbox = serial.Serial(status.split()[0], 115200)
        # Attempt to get data from Arduino

# animate - https://matplotlib.org/stable/api/_as_gen/matplotlib.animation.FuncAnimation.html
ani = FuncAnimation(fig, my_function, interval=0)


plt.show()

