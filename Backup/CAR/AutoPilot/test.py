import serial
import socket

import socket
import sys

import time

# Socket Start
jetson_address = "127.0.0.1"
jetson_port = 5002 # target data port
buffer_size = 1024

jetson_info = (jetson_address, jetson_port)

client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Socket End

count = 0
while True:
	print("Sent: " + str(count))
	count += 1
	YawAngle_data_msg = str.encode(str(count))
	client_socket.sendto(YawAngle_data_msg, jetson_info)
	time.sleep(0.01)

