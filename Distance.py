import RPi.GPIO as GPIO
import time

def config_dtc(TRIG, ECHO):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

def measure_distance(TRIG, ECHO):
    # Đảm bảo Trig bắt đầu ở mức thấp
    GPIO.output(TRIG, False)
    time.sleep(0.1)  # Chờ cảm biến ổn định

    # Gửi xung 10µs đến chân Trig
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Đo thời gian từ khi xung bắt đầu đến khi Echo nhận được tín hiệu
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    # Tính toán khoảng cách
    distance = pulse_duration * 17150  # Tốc độ âm thanh là 34300 cm/s, chia 2 vì xung đi và về
    distance = round(distance, 2)

    return distance

