import socket
import sys

jetson_address = "127.0.0.1"
jetson_port = 5001
buffer_size = 1024

jetson_info = (jetson_address, jetson_port)

control_data = 0

client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# steering, velocity, break
control_data = "0_0_0"

while 1:

    control_data = str(80) + "_" + str(90) + "_" + str(80) + "_"
    control_data_msg = str.encode(str(control_data))
    client_socket.sendto(control_data_msg, jetson_info)

    msg_from_jetson = client_socket.recvfrom(buffer_size)
    msg = "Messsage from Jetson {}".format(msg_from_jetson[0])

    print(control_data)
    #print(msg)
