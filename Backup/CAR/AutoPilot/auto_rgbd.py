import cv2
import math
import numpy as np

import socket
import sys
sys.path.append('/home/canlab-jetson-1/opencv/build/lib')

# Socket Start
jetson_address = "127.0.0.1"
jetson_port = 5001
buffer_size = 1024

jetson_info = (jetson_address, jetson_port)

control_data = 0

client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Socket End

def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #lower_blue = np.array([20, 20, 20])
    #upper_blue = np.array([150, 150, 150])
    #lower_black = np.array([0, 0, 0])
    #upper_black = np.array([80, 80, 80])
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask_1 = cv2.inRange(hsv, lower_red, upper_red)

    #lower_red = np.array([170, 50, 50])
    #upper_red = np.array([180, 255, 255])
    #mask_2 = cv2.inRange(hsv, lower_red, upper_red)

    #mask = mask_1 + mask_2
    mask = mask_1
    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    polygon = np.array([[
        (0, height * 1/2),
        (width, height * 1/2),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)

    return line_segments

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        print('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/2
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 3)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def detect_lane(frame):
	#cv2.imshow('Input', frame)
	edges = detect_edges(frame)
	#cv2.imshow('Input', edges)
	cropped_edges = region_of_interest(edges)
	#cv2.imshow('Input', cropped_edges)
	line_segments = detect_line_segments(cropped_edges)
	lane_lines = average_slope_intercept(frame, line_segments)
	lane_lines_image = display_lines(frame, lane_lines)
	
	return lane_lines, lane_lines_image

def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        print('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        print('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.00 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = ((left_x2 + right_x2) / 2 - mid) * 0.4
	# x_offset adjusting
	#x_offset = x_offset * 0.4

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    #print('new steering angle: %s' % steering_angle)
    return steering_angle

def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    #print('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle


cap = cv2.VideoCapture(1)
cap.set(3, 640)
cap.set(4, 480)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# initial steering angle
curr_steering_angle = 80

# steering, velocity, break
control_data = "0_0_0_"

while True:
	ret, frame = cap.read()
	#frame = cv2.resize(frame, (600, 400))
	#frame = frame[50:300,:]
	lane_lines, frame = detect_lane(frame)
	new_steering_angle = compute_steering_angle(frame, lane_lines)
	curr_steering_angle = stabilize_steering_angle(curr_steering_angle, new_steering_angle, len(lane_lines))
	#curr_steering_angle= new_steering_angle
	
	cv2.imshow('Input', frame)

	c = cv2.waitKey(1)
	if c == 27:
		break

	if curr_steering_angle > 125:
		curr_steering_angle = 125
	elif curr_steering_angle < 55:
		curr_steering_angle = 55

	# Data Sending
	control_data = str(curr_steering_angle-10) + "_" + str(88) + "_" + str(80) + "_" + "1"
	control_data_msg = str.encode(str(control_data))
	client_socket.sendto(control_data_msg, jetson_info)

	#msg_from_jetson = client_socket.recvfrom(buffer_size)
	#msg = "Messsage from Jetson {}".format(msg_from_jetson[0])

	print(control_data)

cap.release()
cv2.destroyAllWindows()
