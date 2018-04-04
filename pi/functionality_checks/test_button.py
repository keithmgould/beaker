# External module imports
import RPi.GPIO as GPIO
import time

# Pin Definitons:
butPin = 4 # Broadcom pin 4 (P1 pin 7)


# Pin Setup:
GPIO.setmode(GPIO.BCM)
GPIO.setup(butPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Button pin set as input w/ pull-up

# Initial state for LEDs:

print("Here we go! Press the button on Beaker! Press CTRL+C to exit")
try:
    while 1:
        if GPIO.input(butPin): # button is released
          a = 1
           # do Nothing
        else: # button is pressed:
          print("button pressed!")
          time.sleep(0.075)
except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
    GPIO.cleanup() # cleanup all GPIO
