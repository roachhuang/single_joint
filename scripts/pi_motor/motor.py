import Motor    # imports our motor.py class
import time

# Instantiates a motor using pins 7, 11
lmotor = motor.motor(7, 11)
rmotor = motor.motor()

lmotor.stop()
time.sleep(2) # waits for 2 seconds 
rmotor.stop()



