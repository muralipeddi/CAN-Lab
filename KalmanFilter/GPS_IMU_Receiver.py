#------------------------------------
# IMU GPS Receiver
#------------------------------------

import serial
import math

def package_decode(_package):
	# error filter
	if len(_package) < 6:
		return []

	# remove header
	newPackage = _package.replace('ff47', '')

	# package type
	new_package_type = newPackage[:4]
	newPackage = newPackage[4:]

	# IMU Type 1
	if new_package_type == "0500":
		# data size
		new_package_size = newPackage[:2]
		new_package_size = int(new_package_size, 16)
		newPackage = newPackage[2:]

		# error filter
		if not len(newPackage) == 86:
			return []

		# Coordinate
		new_package_coordinate = newPackage[:24]
		newPackage = newPackage[24:]

		# Quaternion
		new_package_compass_w = newPackage[:4]
		new_package_compass_w = negative_hex(int(inverse_2_digits(new_package_compass_w), 16))
		newPackage = newPackage[4:]
		new_package_compass_x = newPackage[:4]
		new_package_compass_x = negative_hex(int(inverse_2_digits(new_package_compass_x), 16))
		newPackage = newPackage[4:]
		new_package_compass_y = newPackage[:4]
		new_package_compass_y = negative_hex(int(inverse_2_digits(new_package_compass_y), 16))
		newPackage = newPackage[4:]
		new_package_compass_z = newPackage[:4]
		new_package_compass_z = negative_hex(int(inverse_2_digits(new_package_compass_z), 16))
		newPackage = newPackage[4:]

		# Velocity
		new_package_velocity = newPackage[:12]
		newPackage = newPackage[12:]

		# Acceleration
		new_package_acceleration_x = newPackage[:4]
		newPackage = newPackage[4:]
		new_package_acceleration_y = newPackage[:4]
		newPackage = newPackage[4:]
		new_package_acceleration_z = newPackage[:4]
		newPackage = newPackage[4:]

		# Address
		new_package_address = newPackage[:2]
		print(new_package_address)
		new_package_address = int(inverse_2_digits(new_package_address), 16)
		newPackage = newPackage[2:]

		#if not (new_package_address == 192):
		#	return []

		# Angle Calculation
		qw = new_package_compass_w
		qx = new_package_compass_x
		qy = new_package_compass_y
		qz = new_package_compass_z
		print(str(new_package_address) + "\tQ: " + str(qw) + " \t " + str(qx) + " \t " + str(qy) + " \t " + str(qz))
		qnorm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
		qw = qw / qnorm
		qx = qx / qnorm
		qy = qy / qnorm
		qz = qz / qnorm

		yaw_angle = math.atan2(2 * (qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
		pitch_angle = math.asin(2 * (qw*qy - qx*qz) / (qw*qw + qx*qx + qy*qy + qz*qz))
		roll_angle = math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

		#print(roll_angle)
		#print("Angle: " + str(yaw_angle) + " \t " + str(pitch_angle) + " \t " + str(roll_angle))

		# Adjust
		roll_angle = math.pi - roll_angle

		return [new_package_type, roll_angle]

	# Coordinate
	elif new_package_type == "1100":
		# data size
		new_package_size = newPackage[:2]
		new_package_size = int(new_package_size, 16)
		newPackage = newPackage[2:]

		# Timestamp
		new_package_timestamp = newPackage[:8]
		new_package_timestamp = int(inverse_2_digits(new_package_timestamp), 16)
		newPackage = newPackage[8:]

		# error filter
		if len(newPackage) < 28:
			return []

		# X
		new_package_x = newPackage[:8]
		new_package_x = negative_hex(int(inverse_2_digits(new_package_x), 16))
		newPackage = newPackage[8:]

		# Y
		new_package_y = newPackage[:8]
		new_package_y = negative_hex(int(inverse_2_digits(new_package_y), 16))
		newPackage = newPackage[8:]

		# Z
		new_package_z = newPackage[:8]
		new_package_z = negative_hex(int(inverse_2_digits(new_package_z), 16))
		newPackage = newPackage[8:]

		# Flags
		new_package_flags = newPackage[:2]
		newPackage = newPackage[2:]

		# Address
		new_package_address = newPackage[:2]
		new_package_address = int(inverse_2_digits(new_package_address), 16)
		newPackage = newPackage[2:]

		return [new_package_type, new_package_address, new_package_x, new_package_y, new_package_z, new_package_timestamp]
	else:
		return [new_package_type]

def inverse_2_digits(_data):
	new_data = ""
	while len(_data) >= 2:
		new_data += _data[-2:]
		_data = _data[:-2]
	return new_data

def negative_hex(_value):
	return -(_value & 0x8000) | (_value & 0x7fff)

# Constants
ser = serial.Serial('/dev/ttyACM0', 500000)

# Intialize Variables
exceptionalCode = {
	'""'	: "22",
	'\\\\'	: "08",
	'\\t'	: "09",
	'\\n'	: "0A",
	'\\r'	: "0D"
	}
newString = ""
order = 0

while True:
	readedText_raw = ser.read(1)

	readedText = repr(readedText_raw).replace('\'', '', 2)
	readedText = readedText.replace('b', '')

	if '\\x' in readedText:
		newString += readedText.replace('\\x',"")
	else:
		if len(readedText) == 1:
			newString += format(ord(readedText), 'x')
		else:
			if readedText in exceptionalCode:
				readedText = exceptionalCode[readedText]
				newString += readedText
			else:
				print(readedText)
	if 'ff47' in newString:
		newData = package_decode(newString)
		
		#print(order)

		if (len(newString) < 1):
			newString = ""
		else:
			order += 1
		
		newString = ""
