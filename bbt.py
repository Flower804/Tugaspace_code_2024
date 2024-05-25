#PY imports
import serial
import csv
import os
import time
import multiprocessing as mp
from multiprocessing import Manager
from datetime import datetime

cansat imports
import adafruit_bmp3xx
from picamera import PiCamera
import board

#NOTES
#HIIIII I'm Flower
#for references.
#lines where Serial connection is referenced/defined. 54, 97, 128

#handlers
force = True
Turn  = False
data = ("milisec", "acel_x", "acel_y", "acel_z", "acel_tot", "temp", "pressure", "humidity", "altitude", "lumi", "cur_state", "gps_alt", "vel", "dir")
last_read = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) #list for recov sist 
#first time deff/code and log sistem for debugging prop
filename = ('log.txt') #action log sistem
provser = ('provser.txt') #ser log sistem
f = open(filename, 'a')
s = open(provser, 'a')
#def csv file
with open('datasheet.csv', 'a', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter= ' ', quotechar = '|')
    writer.writerow([data])
    
#def I2C connect and backup sensors
i2c = board.I2C()
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)


def get_time(): #get current time for multi use
    current_time = f'{datetime.now(): %H_%M_%S}'
    return current_time

def info_dump(mode, last_read):
    f = open(filename, 'a')
    s = open(provser, 'a') #is this really necessary? :\
    ser_list = [] #WHY AREN'T YOU OK???'
    f.write("-------" + get_time() + "-------" + '\n' + '\n')
    f.close()
    if mode == 1: #mode 1: connection with ESP32 secured. mode2: connection was not sucessful 
        print("data saver using Serial mode")
        while(1):
            f = open(filename, 'a')
            ser = serial.Serial('COM6')
            input_str = ser.readline().decode("utf-8").strip()
            if input_str == "sent_complete":
                serv = ' '.join(ser_list)
                with open('testopt.csv', 'a', newline= '') as csvfile: #honestly I'm  surprised this works
                    writer = csv.writer(csvfile, delimiter= ' ', quotechar = '|')
                    writer.writerow([ser_list])   
                ser.write(b'done\n')
                print("csv done")
                return ser_list
            else:
                f.write("input" + "!" + input_str + '\n')
                f.close()
                inputser = tupleconv(input_str)
                ser_list.append(inputser)
                print("data received")
    else:
        print("data saver using backup mode")
        back_pressure = bmp.pressure #use backup sensors
        back_temperature = bmp.temperature
        back_alt = bmp.altitude
        ser_list = ['N/C', 'N/C', 'N/C', 'N/C', 'N/C', back_temperature, back_pressure, 'N/C', back_alt, 'N/C', 'N/C', 'N/C', 'N/C', 'N/C'] #N/C no sensor available
        with open('testopt.csv', 'a', newline= '') as csvfile: #honestly I'm  surprised this works
                    writer = csv.writer(csvfile, delimiter= ' ', quotechar = '|')
                    writer.writerow([ser_list])  
        print("backup data upload done")
        return last_read
            
def tupleconv(tup): #Done: i think this is necessary 10.1.2.116
    str = ''        #Yup it is
    for item in tup:
        str = str + item
    return str #this does magic



#Multiprocessing protocols
def life_line_protocol(handlers, last_read):
    Tries = 0
    life_strip = False #conection indicator, True: connection was succesful, False: connection was not secured
    print("life line protocol started")
    while(1):
        try: #verify if connection is possible
            print("try section started")
            try:
                ser = serial.Serial('COM4', 9600, timeout=5)
                ser_conectser = ser.readline()
                print("possible serial connection detected")
                #ser.write(b'alive\n')
                print("connection accepted via serial")
                handlers[1] = 1
                return(handlers[1])
            except:
                print("trying already connected serial")
                ser.write(b'alive\n')
                strip_line = ser.readline().decode("utf-8").strip()
                if strip_line == "alive":
                    handlers[1] = 1
                    return(handlers[1])
        except: #if error do this part of the code
            print("connection with esp32 has failed, retrying....")
            Tries = Tries + 1
            if (Tries >= 5):
                print("connection with esp32 has failed too many times changing to backup def")
                life_strip = False
                handlers[1] = 2
                return(handlers[1])
            time.sleep(0.2)


filename = ('sensors_data.txt')
f = open(filename, 'a')
def communication_protocol(self): #serial controler for 2way comm betwhen esp32 and RP
    last_read = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    force = False
    handlers = [0, 0]
    while(1):
        print("comm protocol started")
        handlers[1] = life_line_protocol(handlers, last_read)
        print(handlers[1])
        print("got out of line protocol")
        if (handlers[1] == 1):
            print("communication_protocol started with mode 1")
            ser = serial.Serial('COM4', 9600, timeout=5)
            input_str = ser.readline().decode("utf-8").strip()
            print(input_str) #TODO only for debuggin purposes
            if input_str == "ready" or force == True: #force for when comm is stuck or desicronissed
                print("received ready")
                while(True): 
                    ser.write(b'on\n') #testing purposes
                    input_str = ser.readline().decode("utf-8").strip()
                    if(input_str == "received"): #start csv data sheet
                        last_read = info_dump(handlers[1], last_read)
                        break
            elif(input_str == "ser_update"):
                ser.write(b'starting_comp\n')
                input_str = ser.readline().decode("utf-8").strip()
                if(input_str == "ready_for_opp"):
                    for i in last_read:
                        ser.write(last_read(i) + b'\n')
                        time.delay(0.2)
                ser.write(b'done\n')
            elif(input_str == "."):
                force = True
            elif(input_str == "sent_complete"):
                ser.write(b'done\n')
        else:
            print("communication protocol started with mode 2 :(")
            info_dump(handlers[1], last_read)


def camera_protocol(self): #the name says it all
    a = 0
    cam = PiCamera()
    cam.resolution = (360,260)
    print("cam process initiated")
    while(1):
        a = a + 1 
        cam.start_preview()
        cam.capture('/home/admin/fotos/foto%s.jpg' % a) #save photo file in format foto + photo number + .jpg
        print(bmp.pressure)
        print(bmp.temperature)
        print(bmp.altitude)
        cam.annotate_text = "pressure:" + str(round(bmp.pressure , 2)) + "temp:" + str(round(bmp.temperature , 2 )) + "altitude:" + str(round(bmp.altitude , 2))
        time.sleep(2)
        cam.stop_preview()
        time.sleep(1)
    print("camera took a photo")
    time.sleep(3)

while(__name__ == "__main__"): #multiprocess controler
    print("process started")
    p1 = mp.Process(target=communication_protocol, args=(10, )) #def multprocss for comm prot
    p2 = mp.Process(target=camera_protocol, args=(10, )) #def multprocess for cam prot
    p1.start() #start comm
    print("p1 succ")
    p2.start() #cam start
    print("p2 succ")
    p1.join() #wait untill comm prot finishes
    p2.join() #wait until cam procc finishes