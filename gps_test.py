import bluetooth 

# hostMACAddress = 'C0:D2:DD:47:69:BA' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters. 
# hostMACAddress = '18:19:D6:DD:C6:A4' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters. 
hostMACAddress = 'a8:6d:aa:eb:50:7c' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters. 
port = 0
backlog = 1
size = 1024
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.bind((hostMACAddress, port))
s.listen(backlog)
try:
	client, clientInfo = s.accept()
	while 1:
		data = client.recv(size)
		if data:
			print(data)
			client.send(data) # Echo back to client
except:	
	print("Closing socket")
	client.close()
	s.close()