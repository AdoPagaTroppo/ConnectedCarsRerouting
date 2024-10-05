# Script for saving GPS locations for testing vehicle-in-the-loop simulations

from time import sleep
from pynmeagps import NMEAReader
from threading import Thread
import serial

# thread for reading data from the serial port, inputs are:
# - the serial port
def reading_thread(ser):
    global msg
    print('thread is on')
    msg = None
    try:
        nmr = NMEAReader(ser)
        while True:
            (raw_data, msg2) = nmr.read() #msg will be global variable that main will read
            msgstr = str(msg2)
            print(msgstr)
            if msgstr.__contains__('lat=') and msgstr.__contains__('lon=') and (not msgstr.__contains__('lat=,') and not msgstr.__contains__('lon=,')):
                msg = msg2
                print(f"lat: {msg.lat} - lon: {msg.lon}")
                f = open('log_gps_test.txt','a')
                f.write(str(msg.lat)+':'+str(msg.lon)+'\n')
                f.close()
    except KeyboardInterrupt:
        ser.close()

ser = serial.Serial('/dev/rfcomm0', 9600)
    
f = open('log_gps_test.txt','w') # reset content of 'log_gps_test.txt' file
f.close()
reading_thread(ser) # start reading data from serial port