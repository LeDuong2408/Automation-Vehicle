import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import imutils
import gpiozero
import RPi.GPIO as GPIO
import math
import Distance as dtc
import Ledutils as Lutils

camera = PiCamera()
camera.resolution = (640,368)  # Adjusted to the rounded size
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,368))

TRIG = 21
ECHO = 20
robot = gpiozero.Robot(left=(24,23), right=(17,27))
forward_speed = 0.65
backward_speed = 0.5
#turn_speed = 0.55

dtc.config_dtc(TRIG,ECHO)


# Greyscale
def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    canny = cv2.Canny(blur, 50,150)
    return canny

# Region of interest
def region_of_interest(image):
    height, width = image.shape # to know the height       
    polygons = np.array([
    [(0, height), (width,height), (width,150),(0,150)]
    ]) # our array of polygons
    mask = np.zeros_like(image) # creates an array of zeros with the same shape as image
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

# Display the lines

def display_lines(image, lines):
    height, width, _ = image.shape
    line_image = np.zeros_like(image)	
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # Ensure the points are integers
            if x1 > 2000000000 :
                x1 = 2000000000
            if x2 > 2000000000 :
                x2 = 2000000000
            if x1 < -2000000000:
                x1 = -2000000000
            if x2 < -2000000000 :
               x2 = -2000000000
            print(f'x1: {x1}, y1: {y1}, x2: {x2}, y2" {y2}')
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_image


# Average of the lines
def average_slope_intercept(image, lines):
	height, width, _ = image.shape
	lane_lines = []
	left_fit = []
	right_fit = []
	if lines is None:
	    #x1 = int(width / 2)
	    #y1 = height
	    #x2 = int(width / 2)
	    #y2 = int(height / 2)
	    #lane_lines.append(np.array([x1, y1, x2, y2]))
	    return lane_lines
	for line in lines:
		x1, y1, x2, y2 = line.reshape(4)
        # We get the parameters m and b of a one-order line to fit these 2 points
		parameters = np.polyfit((x1,x2), (y1,y2), 1)
		slope = parameters[0]
		intercept = parameters[1]
        # y is reversed => left line has negative slope
		if slope<0:
			left_fit.append((slope,intercept))
		else:
			right_fit.append((slope,intercept))
            # Then we get the average slope
	left_fit_average = np.average(left_fit, axis = 0)
	right_fit_average = np.average(right_fit, axis = 0)
    # We found the parameters but then we need to translate them into x and y
	if len(left_fit) > 0:
		lane_lines.append(make_coordinates(image, left_fit_average))
	if len(right_fit) > 0:
		lane_lines.append(make_coordinates(image, right_fit_average))
	return lane_lines
    # Make x, y coordinates
def make_coordinates(image, line_parameters):
    height, width, _ = image.shape
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int((y1)*(175/368))
    x1 = ((y1 - intercept)/slope)
    x2 = ((y2 - intercept)/slope)
    if x1 > 2000000000 :
                x1 = 2000000000
    if x2 > 2000000000 :
                x2 = 2000000000
    if x1 < -2000000000:
                x1 = -2000000000
    if x2 < -2000000000 :
               x2 = -2000000000
    x1 = int(x1)
    x2 = int(x2)
    return np.array([x1, y1, x2, y2])

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape
    x_offset, y_offset = 0,int(height / 2)
    if lane_lines is None:
        x_offset = 0
        y_offset = int(height / 2)
        print('Noneee Linee')
    elif len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1] # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        print('Twwo Lineeee')
    elif len(lane_lines) == 1: # if only one line is detected
        print('One Lineeeeeeeeeeeeee!!!')
        x1, _, x2, _ = lane_lines[0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90
    print(steering_angle)

    return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):

    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - (height / 2 / math.tan(steering_angle_radian)))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def avoid_object():
        distance = dtc.measure_distance(TRIG,ECHO)
        print('Distance: ', distance)
        if distance <= 50:
                robot.stop()
def control_robot(steering_angle):
	Lutils.led_lightsensor()
	turn_speed = 0.75
	distance = dtc.measure_distance(TRIG,ECHO)
	print('Distance: ', distance)
	if distance <= 15:
		robot.stop()
		Lutils.led_stop()
		Lutils.buzzer_alarm()
		print('Stop')
	
	elif steering_angle < 85:
		if steering_angle < 45:
			turn_speed = 0.85
		robot.left(turn_speed)
		#Lutils.led_turnleft()
		print("Turn Left!")
				
	elif steering_angle > 95:
		if steering_angle > 135:
			turn_speed = 0.85
		robot.right(turn_speed)
		#Lutils.led_turnright()
		print("Turn Right")
	else:
				robot.forward(forward_speed)
				notfound = 0
				print("Foward!")

    
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = frame.array # returns a boolean we are not interested now, and the frame
    canny_image = canny(frame)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    averaged_lines = average_slope_intercept(frame, lines)
    line_image = display_lines(frame, averaged_lines)
    combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    steering_angle = get_steering_angle(frame, averaged_lines)
    control_robot(steering_angle)
    heading_image = display_heading_line(combo_image,steering_angle)
    cv2.imshow("result", heading_image)

    rawCapture.truncate(0)
    # Displays the image for a specific amount of time
    # With 1 it will wait 1ms in between the frames. Ιf we press q, then break
    if cv2.waitKey(1) == ord('q'):
        break

robot.stop()
cv2.destroyAllWindows()
