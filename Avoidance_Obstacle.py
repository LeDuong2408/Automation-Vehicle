import RPi.GPIO as GPIO                    #Import GPIO library
import time
import Distance as ds
import gpiozero
#Import time library
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)                    # programming the GPIO by BCM pin numbers

TRIG = 21
ECHO = 20
robot = gpiozero.Robot(left=(24,23), right=(17,27))
forward_speed = 0.75
backward_speed = 0.4
turn_speed = 0.7
ds.config_dtc(TRIG,ECHO)
robot.stop()

def clear_break(robot, distance):
	flag=0
	flag_turn = 0
	flag_1 = 0
	count_forward = 0
	d = distance
	while count_forward < 3:
		if flag == 0:
		 robot.right(turn_speed)
		 print('Right')
		 flag = 1
		 flag_turn = 1
		 flag_1 = 1
		 time.sleep(0.75)
		 d = ds.measure_distance(TRIG, ECHO)
		elif flag == 1:
		 robot.left(turn_speed)
		 print('Left ')
		 flag = 0
		 flag_turn = 1
		 time.sleep(0.75)
		 if flag_1 == 1:
		  time.sleep(0.75)
		  flag_1 = 0
		 d = ds.measure_distance(TRIG, ECHO)
		if flag_turn == 1:
			if d <= 25:
				count_forward = 0
				flag=0
				flag_turn = 0
				flag_1 = 0
			else:
			 robot.forward(forward_speed)
			 count_forward= count_forward +1
			 flag_turn = 0
			 time.sleep(1.5)
			
		robot.stop()
		time.sleep(1)

try: 
	while True:
	    robot.forward(forward_speed)
	    distance = ds.measure_distance(TRIG, ECHO)
	    if distance <= 25:
	    	    robot.stop()
	    	    time.sleep(1)
	    	    clear_break(robot, distance)
	    time.sleep(0.1)
except KeyboardInterrupt:
    print("Chương trình dừng lại.")
    cv2.destroyAllWindows()

	
