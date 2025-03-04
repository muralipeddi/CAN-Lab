import numpy as np
import sys

import serial
import binascii

import socket
import math

import time as tt
import os

import optparse
import subprocess
import random

from numpy.linalg import inv
import numpy as np 
import numpy.matlib
from numpy import linalg as LA

# Socket Start
jetson_address = "127.0.0.1"
jetson_port = 5001
buffer_size = 1024

jetson_info = (jetson_address, jetson_port)

control_data = 0

client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Socket End

ser = serial.Serial('/dev/ttyACM0', 500000)
ser_imu = serial.Serial('/dev/ttyUSB0', 115200)

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
	#if new_package_type == "0500":
	if False:
		# data size
		new_package_size = newPackage[:2]
		new_package_size = int(new_package_size, 16)
		newPackage = newPackage[2:]

		# error filter
		if len(newPackage) < 66:
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
		new_package_address = int(inverse_2_digits(new_package_address), 16)
		newPackage = newPackage[2:]

		qw = new_package_compass_w
		qx = new_package_compass_x
		qy = new_package_compass_y
		qz = new_package_compass_z
		yaw_angle = math.atan2(2 * (qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
		pitch_angle = 0#math.asin(-2*(qx*qz - qw*qy))
		roll_angle = math.atan2(2 * (qw*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

		# Adjust
		roll_angle = math.pi - roll_angle

		return [new_package_type, roll_angle]

	# Coordinate
	elif new_package_type == "1100":
		if len(newPackage) < 1:
			return []
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

		print([new_package_type, new_package_address, new_package_x, new_package_y, new_package_z, new_package_timestamp])

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

#############################################
# Kalman
x_0 = np.array([20, 0])  # position and velocity
P_0 = 1 * np.eye(2)
x_esti, P = None, None

def kalman_filter(z_meas, x_esti, P, dt):
	"""Kalman Filter Algorithm."""
	A = np.array([[1, dt],
		      [0, 1]])
	H = np.array([[1, 0]])
	Q = np.array([[1, 0],
		      [0, 3]])
	R = np.array([[10]])

	# (1) Prediction.
	x_pred = np.matmul(A, x_esti)
	P_pred = np.matmul(np.matmul(A, P), A.T) + Q

	# (2) Kalman Gain.
	K = np.matmul(np.matmul(P_pred, H.T), inv(np.matmul(np.matmul(H, P_pred), H.T) + R))

	# (3) Estimation.
	x_esti = x_pred + np.matmul(K, (z_meas - np.matmul(H, x_pred)))

	# (4) Error Covariance.
	P = P_pred - np.matmul(np.matmul(K, H), P_pred)

	return x_esti, P
#############################################


tempYawDataList = []
timer = tt.time()
pre_theta = 0
pre_error = 0
collected_data_index = 0
collected_data_x = ""
collected_data_u = ""
rank_theta = 0
data_y = 0
newYawData = 0
error = 0

#############################
# Attack
attack_list_index = 0
real_data_count = 0
prev_x = None

#attack_list = [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0]
#attack_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,]
#attack_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,]
attack_list = []
#############################

collected_data_pos = ""

# Return: [isSend(T/F), Steer, Velocity]
def data_processing(_newData):
	if len(_newData) < 2:
		return [False, 0, 0]

	global tempYawDataList
	global virtual_X
	global virtual_Y
	global timer
	global pre_theta
	global pre_error
	global rank_theta
	global data_y
	global error

	global YawAngle
	global newYawData

	newType = _newData[0]
	theta = 0
	error = 0

	if newType == "0500":
		newYawData = _newData[1]
		tempYawDataList.append(newYawData)
	elif newType == "1100":
		if len(tempYawDataList) == 0:
			return [False, 0, 0]
		
		newAddress = _newData[1]
		newX = _newData[2]
		newY = _newData[3]
		newZ = _newData[4]

		dt = tt.time() - timer
		timer = tt.time()

		#if collected_data_index > 101:
		#	virtual_Y = -3000
		data_y = newY

		#theta = sum(tempYawDataList)/len(tempYawDataList) - math.pi/2
		#theta = tempYawDataList[-1]
		#theta = YawAngle
		theta = -1* newYawData
		#print("Current Angle: " +str(theta))
		error = -1*(newY - virtual_Y) * 1.0 / 10 / 100 # mm -> cm -> m

		standard_steering = 75 # 85
		standard_speed = 92
		#kalman_sample_size = 20
		kalman_sample_size = 0
		sample_size = 40
		max_data_size = 200
		isLearning = True

		sample_size = sample_size + kalman_sample_size
		max_data_size = max_data_size + kalman_sample_size

		#error = error * -1
		if dt == 0:
			return [False, 0, 0]

		print('dt: ', dt)
		theta_dot = 1*(pre_theta - theta) / dt# * 0.01
		error_dot = -1*(pre_error - error) / dt# * 0.01

		#if error < 5:
		#	error = 0
		#	error_dot = 0

		# update		
		pre_theta = theta
		pre_error = error
		tempYawDataList = []

		##########################################
		# Lane changing algorithm - Start		
		global collected_data_index
		global collected_data_x
		global collected_data_u
		global collected_data_pos

		# Attack
		global attack_list_index
		global real_data_count
		global attack_list
		global prev_x

		global x_0
		global P_0
		global x_esti
		global P_kalman

		collected_data_index += 1

		###########
		# Kalman - Start
		#if collected_data_index == 1:
		#	x_esti, P_kalman = x_0, P_0
		#else:
		#	z_meas = error
		#	x_esti, P = kalman_filter(z_meas, x_esti, P_kalman, dt)
		#	error, error_dot = x_esti
		# Kalman - End
		###########

		# round
		error = round(error, 4)
		error_dot = round(error_dot, 4)
		theta = round(theta, 4)
		theta_dot = round(theta_dot, 4)

		# data collect
		data_x = str(collected_data_index) + "\t" + str(error) + "\t" + str(error_dot) + "\t" + str(theta) + "\t" + str(theta_dot)

		# data confirming
		x = np.array([[error], [error_dot], [theta], [theta_dot]])

		print('Data Index: ',collected_data_index)
		print('X:\n')
		print(x)

		if collected_data_index <= sample_size and isLearning:
			global x0
			global K
			global Dxx
			global Ixx
			global Ixu
			global Iuu			

			if collected_data_index == 1:
				x0 = x
			##################################################
			attack_check = 0			
			if attack_list_index < len(attack_list):
				attack_check = attack_list[attack_list_index]
			
			print('Attack?: ',attack_check)
			print('Attack index: ', attack_list_index)

			if attack_check == 1:
				collected_data_index -= 1

			attack_list_index += 1
			##################################################

			u = model(x0, collected_data_index, K, 0)
			newSteer = u[0][0]

			newSteer = newSteer / math.pi * 180

			newSteer = newSteer
			if newSteer > 25:
				newSteer = 25
			elif newSteer < -25:
				newSteer = -25

			#u = np.array([[newSteer / 180 * math.pi]])
			print("ANGLE: \t" +str(u))
			if attack_check == 0:
				b0 = np.kron(x.T,x.T)-np.kron(x0.T,x0.T)
				Dxx = np.append(Dxx, b0, axis=0)
				
				b1 =  np.kron(x0.T,x0.T)
				Ixx = np.append(Ixx, b1, axis=0)
				
				b2 =  np.kron(x0.T,u.T)
				Ixu = np.append(Ixu, b2, axis=0)
				
				b3 =  np.kron(u.T,u.T)
				Iuu = np.append(Iuu, b3, axis=0)
				x0 = x
			elif attack_check == 1:
				x0 = x0

			newSteer = round(standard_steering + newSteer)

			# data collect
			data_x = data_x + "\t" + str(u) + "\n"
			collected_data_x = collected_data_x + data_x
			collected_data_u = collected_data_u + str(collected_data_index) + "\t" + str(newSteer-standard_steering) + "\n"

			return [True, newSteer, standard_speed]
		elif collected_data_index == (sample_size + 1) and isLearning:
			DxxU = Dxx[1:-1,[0,1,2,3,5,6,7,10,11,15]]
			IuuU = Iuu[1:-1,:]
			Ixx = Ixx[1:-1,:]
			Ixu = Ixu[1:-1,:]
			#print(DxxU.shape, Ixx.shape, IuuU.shape, Ixu.shape)
			#print(LA.matrix_rank(DxxU), LA.matrix_rank(Ixx), LA.matrix_rank(IuuU), LA.matrix_rank(Ixu))
			collected_data_x = collected_data_x + data_x + "\n"

			collected_data_x = collected_data_x + str(K) + "\n"

			P,K, learningTime, rank_theta = learning_K(K, DxxU, Ixx, Ixu, IuuU)

			# data collect
			K_pure = K[0]
			K_0 = K_pure[0]
			K_1 = K_pure[1]
			K_2 = K_pure[2]
			K_3 = K_pure[3]
			
			collected_data_x = str(K_0) + "\t" + str(K_1) + "\t" + str(K_2) + "\t" + str(K_3) + "\n\n" + "order\te\te_dot\ttheta\ttheta_dot\n" + collected_data_x + "\n"

			# save data
			text_file = open("data_Train.txt","wt")
			w_text = text_file.write(collected_data_x)
			text_file.close()

			print(K)
			u = model(x, collected_data_index, K, 0)

			newSteer = u[0][0]
			newSteer = newSteer / math.pi * 180

			newSteer = newSteer

			if newSteer > 25:
				newSteer = 25
			elif newSteer < -25:
				newSteer = -25

			newSteer = round(standard_steering + newSteer)
			collected_data_u = collected_data_u + str(collected_data_index) + "\t" + str(newSteer) + "\n"
			collected_data_u = "order\tsteering\n" + collected_data_u
			
			return [True, newSteer, standard_speed]
		else:
			#K = np.array([[0.0146,   -0.0750,    2.1831,    0.8227]])
			#K = np.array([[0.0157,   -0.06898177,    1.37309969,    0.2375484]])
			#K = np.array([[0.047,   -0.06898177,    1.6309969,    0.00002]])
			#K = np.array([[0.0147,   -0.06898177,    1.0309969,    0.00002]])
			#K = np.array([[0.0127,   -0.06898177,    1.0309969,    0.00002]])
			K = np.array([[0.727,   -0.06898177,    0.7309969,    0.00002]])
			#K = np.array([[0.0107,   -0.06898177,    0.9309969,    0.00002]])

			##################################################
			attack_check = 0			
			if attack_list_index < len(attack_list):
				attack_check = attack_list[attack_list_index]
			
			print(attack_check)

			if attack_check == 0:
				prev_x = x
			elif attack_check == 1:
				x = prev_x
			
			attack_list_index += 1
			
			data_x = data_x + "\t" + str(attack_check) + "\n"

			##################################################

			u = model(x, collected_data_index, K, 1)
			
			# data collect
			collected_data_x = collected_data_x + data_x
			collected_data_pos = collected_data_pos + str(newX) + " " + str(newY) + "\n"

			if collected_data_index == max_data_size:
				# save data
				text_file_x = open("data_Total_attack.txt","wt")
				w_text_x = text_file_x.write(collected_data_x)
				text_file_x.close()

				text_file_u = open("data_Steering_attack.txt","wt")
				w_text_u = text_file_u.write(collected_data_u)
				text_file_u.close()

				text_file_pos = open("data_pos_attack.txt","wt")
				w_text_pos = text_file_pos.write(collected_data_pos)
				text_file_pos.close()

			newSteer = u[0][0]
			print("Angle")
			print(newSteer)
			newSteer = newSteer / math.pi * 180

			if newSteer > 25:
				newSteer = 25
			elif newSteer < -25:
				newSteer = -25

			newSteer = round(standard_steering + newSteer)
			collected_data_u = collected_data_u + str(collected_data_index) + "\t" + str(newSteer) + "\n"
			print(K)
			return [True, newSteer, standard_speed]
			#print((u + math.pi)* 180 / math.pi)
		# Lane changing algorithm - End
		##########################################

	return [False, 0, 0]

##########################################
# Lane Changing - Start
rank_theta = 0
def learning_K(K, Dxx, Ixx, Ixu, Iuu):
    
	epsi = 1e-4;
	it = 0; 
	POld = np.zeros(4)
	KOld = np.zeros([1,4])
	R = 1
	Q = 0.1*np.diag([1.0, 1.0, 1.0, 1.0])
	P = np.diag([1.0, 1.0, 1.0, 1.0])
	learning_start_time = tt.time()
	global rank_theta
	while (LA.norm(P-POld) > epsi) and it<100:
		err = LA.norm(P-POld)
		it = it + 1; 
		KOld = K
		POld = P
		Qk = Q + np.matmul(K.T*R,K)
		Theta = np.append( np.append(Dxx, -2*np.matmul(Ixx,np.kron(np.eye(4),K.T))-2*Ixu, axis=-1) , np.matmul(Ixx,np.kron(K.T,K.T))-Iuu, axis=-1)

		print('learning iteration:',it, 'rank Theta=', LA.matrix_rank(Theta), 'error=', err)
		QkVec = Qk.flatten(order='F').reshape(16,1)
		Xi = np.matmul(-Ixx,QkVec);

		pp = np.matmul(LA.pinv(Theta),Xi);

		P = np.array([[pp[0], pp[1]/2, pp[2]/2, pp[3]/2], 
		   [pp[1]/2, pp[4], pp[5]/2, pp[6]/2],
		   [pp[2]/2, pp[5]/2, pp[7], pp[8]/2],
		   [pp[3]/2, pp[6]/2, pp[8]/2, pp[9]]])
		P =  ReshapeMat(P, 4, 4)

		BPA = pp[10:14].reshape(1,4)
		BPB = pp[14]
		K = np.matmul(1/(R+BPB),BPA)
		K = K.reshape(1,4)
		
	print(P)
	w, v = LA.eig(np.array(P))
	print("Learning Test Start --------------------------")
	print('eig values: ', w)
	print('BPA: ', BPA)
	print('BPB:', BPB)
	learning_end_time = tt.time()
	rank_theta = LA.matrix_rank(Theta)
	print('Theta shape: ', Theta.shape)
	print('learning iteration:',it, 'rank Theta=', LA.matrix_rank(Theta))
	print('learning time:', learning_end_time-learning_start_time)
	print("Learning Test End --------------------------")

	learningTime = learning_end_time-learning_start_time
	return P,K, learningTime, LA.matrix_rank(Theta)

noise_count = 0
pre_noise = 0

def model(x, k, K, ifLEarned):
	if ifLEarned == 0:
		global noise_count
		global pre_noise
		print(noise_count)
		ExpNoise = 0
		if noise_count == 0:
			rand_vec = np.random.rand(1,5)-0.5
			ExpNoise = 0.5*0.2 * np.sum(np.sin(rand_vec*k))
			pre_noise = ExpNoise
		elif noise_count == 1:
			ExpNoise = pre_noise
			noise_count = -1
		else:
			ExpNoise = pre_noise
		print('Noise: ', ExpNoise)
		u = -np.matmul(K,x)+1*ExpNoise ## use as steering input

		noise_count += 1
	else:
		u = -np.matmul(K,x)
    
	return u

x0 = np.array([[1],[0],[1],[0]])
#K = np.array([[0.146,   -0.0750,    2.0831,    0.8227]]) #Opt
K = np.array([[0.727,   -0.06898177,    0.7309969,    0.00002]])
#K = np.array([[0.0047,   -0.0447,    1.002,    0.0002]]) #init
#K = np.array([[0.0,   0.0,    0.0,    0.0]]) #init
#K = np.array([[-0.0246,   0.0450,    -1.0931,    -0.0026]])
#K = np.array([[0.2346,   -0.0750,    0.7831,    0.3227]])
#K = np.array([[0.01934,   -0.07898177,    0.87309969,    0.2375484]])
#K = np.array([[0.0077,   -0.06898177,    0.87309969,    0.2375484]])
Dxx = np.zeros((1,16))
Ixx = np.zeros((1,16))
Ixu = np.zeros((1,4))
Iuu = np.zeros((1,1))

# Lane Changing - End
##########################################

exceptionalCode = {
	'""'	: "22",
	'\\\\'	: "08",
	'\\t'	: "09",
	'\\n'	: "0A",
	'\\r'	: "0D"
	}

newString = ""
newString_imu = ""
order = 0
YawAngle = 0

# Virtual Front Car
virtual_X = 7000
virtual_Y = -2850#-3150
virtual_Y_next_lane = -2950
#virtual_Y_curr_lane = -3150
virtual_Y_curr_lane = -3150
virtual_Y_n_c_center = -3000#(virtual_Y_next_lane + virtual_Y_curr_lane) / 2
gap = 20
AV_lane = 0

# steering, velocity, break
control_data = "0_0_0_"

# Reading UDP -Start------------------------------------------
import threading, queue, time
#jetson_address_target_point = "127.0.0.1" # Localhost
jetson_address_target_point = "0.0.0.0" # Global
jetson_port_target_point = 5002
client_socket_target_point = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
client_socket_target_point.bind((jetson_address_target_point, jetson_port_target_point))
client_socket_target_point.setblocking(0)

target_point = ""

init_yaw_angle = 0
yaw_count = 0
pre_yaw = 0

def Target_Point_UDP():
	global target_point
	while True:
		try:
			data, address = client_socket_target_point.recvfrom(1024)
			data = repr(data)
			data = data.replace('b','')
			data = data.replace('\'','',2)
			target_point = data
		except:
			#print("No Target Point")
			pass

# Target Point
thr = threading.Thread(target=Target_Point_UDP, args=())
thr.start()

def ReshapeMat(_p, _col, _row):
	return _p.reshape(_col,_row)# + np.eye(_col, _row)

# Reading UDP -End------------------------------------------
while True:

	# IMU
	readedText_raw_imu = ser_imu.read(1)
	readedText_imu = repr(readedText_raw_imu).replace('\'', '', 2)
	readedText_imu = readedText_imu[1:]

	if '\\x' in readedText_imu:
		newString_imu += readedText_imu.replace('\\x',"")

	else:
		if len(readedText_imu) == 1:
			newString_imu += format(ord(readedText_imu), 'x')

	if "5561" in newString_imu:
		if len(newString_imu) > 10:
			YawL = newString_imu[-8:-6]
			YawH = newString_imu[-6:-4]

			YawL_int = int(YawL, 16)
			YawL_hex = hex(YawL_int)

			YawH_int = int(YawH, 16)
			YawH_hex = hex(YawH_int)

			YawHL = float((YawH_int<<8) | YawL_int)

			YawHL = negative_hex(int(YawHL))

			YawAngle = float(YawHL)/32768
			YawAngle = YawAngle / math.pi
			if yaw_count == 0:
				init_yaw_angle = YawAngle
				pre_yaw = YawAngle
				yaw_count = 1
			YawAngle = YawAngle - init_yaw_angle
			
			#if abs(YawAngle) >= 0.2:
			#	YawAngle = pre_yaw
			pre_yaw = YawAngle

			#[new_package_type, roll_angle]
			newData = ["0500", YawAngle]

			isSend, newSteer, newVelocity = data_processing(newData)

			if target_point == "2":
				newVelocity = 80

			if isSend:
				# Data Sending
				control_data = str(int(newSteer)) + "_" + str(newVelocity) + "_" + str(80) + "_" + "2"
				control_data_msg = str.encode(str(control_data))
				client_socket.sendto(control_data_msg, jetson_info)
				print(control_data)
		newString_imu = ""

	# GPS
	readedText_raw = ser.read(1)
	readedText = repr(readedText_raw).replace('\'', '', 2)
	readedText = readedText[1:]

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
		isSend, newSteer, newVelocity = data_processing(newData)

		# virtual_Y_next_lane = -2950
		# virtual_Y_curr_lane = -3100
		# virtual_Y_n_c_center = 
		# gap = 30
		# data_y
		# AV_lane

		if data_y <= virtual_Y_n_c_center + gap:
			AV_lane = 0
		elif data_y >= virtual_Y_n_c_center - gap:
			AV_lane = 1


		if target_point == "0": # curr lane
			print("Keep")
			#if AV_lane == 0:
			#	virtual_Y = virtual_Y_curr_lane
			#elif AV_lane == 1:
			#	virtual_Y = virtual_Y_next_lane
		elif target_point == "11": # left next lane
			print("Left Change")
			virtual_Y = virtual_Y_next_lane
			if AV_lane == 0:
				pass#virtual_Y = virtual_Y_next_lane
			elif AV_lane == 1:
				print("STOP")
				virtual_Y = virtual_Y_next_lane
				newVelocity = 80
				
		elif target_point == "12": # right next lane
			print("Right Change")
			if AV_lane == 0:
				print("STOP")
				virtual_Y = virtual_Y_next_lane
				newVelocity = 80
			elif AV_lane == 1:
				continue
				virtual_Y = virtual_Y_curr_lane

		elif target_point == "13": # both next lane
			print("Both Change")
			
			if AV_lane == 0:
				virtual_Y = virtual_Y_next_lane
			elif AV_lane == 1:
				continue
				virtual_Y = virtual_Y_curr_lane

		elif target_point == "2": # stop
			print("Stop")
			newVelocity = 80

		print("Current Target: " + str(virtual_Y))
		print("Current Lane: " + str(AV_lane))

		if isSend:
			# Data Sending
			control_data = str(int(newSteer)) + "_" + str(newVelocity) + "_" + str(80) + "_" + "2"
			control_data_msg = str.encode(str(control_data))
			client_socket.sendto(control_data_msg, jetson_info)
			print(control_data)

			#msg_from_jetson = client_socket.recvfrom(buffer_size)
			#msg = "Messsage from Jetson {}".format(msg_from_jetson[0])

		if (len(newString) < 1):
			newString = ""
		else:
			order += 1
		
		newString = ""

	

cap.release()
cv2.destroyAllWindows()
