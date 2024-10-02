from time import sleep
from pynmeagps import NMEAReader
from threading import Thread
import serial

# hostMACAddress = 'C0:D2:DD:47:69:BA' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters. 
# hostMACAddress = '18:19:D6:DD:C6:A4' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters. 
# hostMACAddress = 'a8:6d:aa:eb:50:7c' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters. 
# port = 0
# backlog = 1
# size = 1024
# s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
# s.bind((hostMACAddress, port))
# s.listen(backlog)
# try:
# 	client, clientInfo = s.accept()
# 	while 1:
# 		data = client.recv(size)
# 		if data:
# 			print(data)
# 			client.send(data) # Echo back to client
# except:	
# 	print("Closing socket")
# 	client.close()
# 	s.close()


def reading_thread(ser):
    global msg
    print('thread is on')
    msg = None
    try:
        nmr = NMEAReader(ser)
        while True:
            (raw_data, msg2) = nmr.read() #msg will be global variable that main will read
            # block for a moment
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
    
# create a thread
f = open('log_gps_test.txt','w')
f.close()
# thread = Thread(target=reading_thread, args=[ser])
# # run the thread
# thread.daemon = True
# thread.start()
reading_thread(ser)