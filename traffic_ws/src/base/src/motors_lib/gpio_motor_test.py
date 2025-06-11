import Jetson.GPIO as GPIO
import time

# BCM mode
GPIO.setmode(GPIO.BOARD)

# Define your current BCM pins for motor 2 (if using BCM mode)
IN1 = 33
IN2 = 31
PWM = 13

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)

print("Setting motor forward for 2 seconds")
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
GPIO.output(PWM, GPIO.HIGH)
time.sleep(2)

print("Stopping motor")
GPIO.output(PWM, GPIO.LOW)
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)

GPIO.cleanup()
