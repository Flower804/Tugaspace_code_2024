#PY imports
import serial
import os
import time
import multiprocessing as mp
from datetime import datetime
#cansat imports
import adafruit_bmp3xx
from picamera import PiCamera
import board


#handlers
force = True
Turn  = False


#Serial Stuff + Board stuff
#ser = serial.Serial('COM5', 9600, timeout=5) #using COM5 for testing purposes on laptop only
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=5) #using /dev/ttyACM0- serial comm for rasp usb connection to esp
conectsec = ser.readline()

i2c = board.I2C()
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)


def get_time(): #get current time for multi use
    current_time = f'{datetime.now(): %H_%M_%S}'
    return current_time

def info_dump():
    f = open(filename, 'a')
    f.write("-------" + get_time() + "-------" + '\n' + '\n')
    f.close()
    while(1):
        f = open(filename, 'a')
        input_str = ser.readline().decode("utf-8").strip()
        if input_str == "sent_complete":
            ser.write(b'done\n')
            return 0
        else:
            f.write(input_str + '\n')
            f.close()
            

filename = ('sensors_data.txt')
f = open(filename, 'a')
def communication_protocol():
    print("camera process initiated")
    while(1):
        input_str = ser.readline().decode("utf-8").strip()
        print(input_str) #TODO only for debuggin purposes
        if input_str == "ready" or force == True: #force for when comm is stuck or desicronissed
            while(True):
                ser.write(b'on\n') #testing purposes
                input_str = ser.readline().decode("utf-8").strip()
                if(input_str == "received"):
                    info_dump()
                    break
        elif(input_str == "."):
            force = True
        elif(input_str == "sent_complete"):
            ser.write(b'done\n')


def camera_protocol():
    a = 0
    cam = PiCamera()
    cam.resolution = (360,260)
    print("cam process initiated")
    while(1):
        a = a + 1 
        cam.start_preview()
        cam.capture('/home/admin/fotos/foto%s.jpg' % a)
        print(bmp.pressure)
        print(bmp.temperature)
        print(bmp.altitude)
        cam.annotate_text = "pressure:" + str(round(bmp.pressure , 2)) + "temp:" + str(round(bmp.temperature , 2 )) + "altitude:" + str(round(bmp.altitude , 2))
        time.sleep(2)
        cam.stop_preview()
        time.sleep(1)
        
if __name__ == "__main__":
    print("task st")
    p1 = mp.Process(target=communication_protocol, args=(10, )) #def multprocss for comm prot
    p2 = mp.Process(target=camera_protocol, args=(10, )) #def multprocess for cam prot
    p1.start() #start comm
    print("p1 succ")
    p2.start() #cam start
    print("p2 succ")
    while(1):
        p1.join() #wait untill comm prot finishes
        p2.join() #wait until cam procc finishes