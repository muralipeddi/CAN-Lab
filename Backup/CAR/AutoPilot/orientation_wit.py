import serial
import socket

import socket
import sys

# Socket Start
jetson_address = "127.0.0.1"
jetson_port = 5002 # IMU data port
buffer_size = 1024

jetson_info = (jetson_address, jetson_port)

client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Socket End

def negative_hex(_value):
	return -(_value & 0x8000) | (_value & 0x7fff)

ser = serial.Serial('/dev/ttyUSB0', 115200)

newString = ""

while True:
	readedText_raw = ser.read(1)

	readedText = repr(readedText_raw).replace('\'', '', 2)

	if '\\x' in readedText:
		newString += readedText.replace('\\x',"")

	else:
		if len(readedText) == 1:
			newString += format(ord(readedText), 'x')
	if "5561" in newString:
		if len(newString) == 40:
			YawL = newString[-8:-6]
			YawH = newString[-6:-4]

			YawL_int = int(YawL, 16)
			YawL_hex = hex(YawL_int)

			YawH_int = int(YawH, 16)
			YawH_hex = hex(YawH_int)

			YawHL = float((YawH_int<<8) | YawL_int)

			YawHL = negative_hex(int(YawHL))

			YawAngle = float(YawHL)/32768*180
			YawAngle_data = str(YawAngle)
			print(YawAngle_data)
			YawAngle_data_msg = str.encode(str(YawAngle_data))
			client_socket.sendto(YawAngle_data_msg, jetson_info)
		newString = ""

