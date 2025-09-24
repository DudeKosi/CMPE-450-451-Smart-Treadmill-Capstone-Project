import RPi.GPIO as GPIO
import time

start_st = 11
stop_st = 13


GPIO.setmode(GPIO.BOARD)
GPIO.setup(start_st, GPIO.OUT)
GPIO.setup(stop_st, GPIO.OUT)

# The start signal is an active low signal, this prevents issues with floating voltages
for i in range(2):
    GPIO.output(start_st, GPIO.LOW)
    time.sleep(10)
    GPIO.output(start_st, GPIO.HIGH)
    time.sleep(10)

GPIO.cleanup()
