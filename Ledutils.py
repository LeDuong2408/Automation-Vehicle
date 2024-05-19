# This Raspberry Pi code was developed by newbiely.com
# This Raspberry Pi code is made available for public use without any restriction
# For comprehensive instructions and wiring diagrams, please visit:
# https://newbiely.com/tutorials/raspberry-pi/raspberry-pi-light-sensor-led


import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the PIN numbers for the light sensor and the LED
LIGHT_SENSOR_PIN = 4
LED_PIN = 16
BUZZER_PIN = 12
LEFT_LED_BEHIND = 2
RIGHT_LED_BEHIND =3
# Set up the light sensor PIN as an input with a pull-up resistor
GPIO.setup(LIGHT_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set up the LED PIN as an output
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(LEFT_LED_BEHIND, GPIO.OUT)
GPIO.setup(RIGHT_LED_BEHIND, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

def led_lightsensor():
	light_state = GPIO.input(LIGHT_SENSOR_PIN)

        # Control the LED based on the light sensor stat
	if light_state == GPIO.LOW:
            # Light is present, turn off the LED
            GPIO.output(LED_PIN, GPIO.LOW)
	else:
            # Light is not present, turn on the LED
            GPIO.output(LED_PIN, GPIO.HIGH)
def buzzer_alarm():
	GPIO.output(BUZZER_PIN, GPIO.HIGH)
	time.sleep(0.2)
	GPIO.output(BUZZER_PIN, GPIO.LOW)

def led_stop():
	GPIO.output(LEFT_LED_BEHIND, GPIO.HIGH)
	GPIO.output(RIGHT_LED_BEHIND, GPIO.HIGH)
	time.sleep(0.2)
	GPIO.output(LEFT_LED_BEHIND, GPIO.LOW)
	GPIO.output(RIGHT_LED_BEHIND, GPIO.LOW)
def led_turnleft():
	GPIO.output(LEFT_LED_BEHIND, GPIO.HIGH)
	time.sleep(0.2)
	GPIO.output(LEFT_LED_BEHIND, GPIO.LOW)
def led_turnright():
	GPIO.output(RIGHT_LED_BEHIND, GPIO.HIGH)
	time.sleep(0.2)
	GPIO.output(RIGHT_LED_BEHIND, GPIO.LOW)
	
