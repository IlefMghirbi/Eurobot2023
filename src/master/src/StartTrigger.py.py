#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
JACK_PIN = 6
GPIO.setup(JACK_PIN, GPIO.IN,GPIO.PUD_UP)

# Set up edge detection
GPIO.add_event_detect(JACK_PIN, GPIO.RISING,bouncetime=200)

# Wait for rising edge
while True:
    if GPIO.event_detected(JACK_PIN):
        print("Rising edge detected!")
    
    time.sleep(0.01)