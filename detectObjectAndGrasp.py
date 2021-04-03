import cv2
import os
import numpy as np
import imutils
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time

# Define pin alloc
trig = 16
echo = 18

def distance():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    
    #Ensure outout has no value
    GPIO.output(trig, False)
    time.sleep(0.01)

    #Generate Trigger pulse
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    #Generate Echo time signal
    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    #Convert time to distance
    distance = pulse_duration*17150
    distance = round(distance, 2)
        
    #clear the output pins
    #GPIO.cleanup()
        
    return distance

# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))

# allow the camera to warmup
time.sleep(0.1)

# setup the GPIO Pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(36, GPIO.OUT)
pwm = GPIO.PWM(36, 50)

# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('DectectAndGrasp.avi', fourcc, 10, (640, 480))

rate = 0.15
duty = float(6.25)
counter = 0 #open
pwm.start(6.25)
# write frame to video file
# keep looping
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
    
    # grab the current frame
    image = frame.array
    image = cv2.rotate(image, cv2.ROTATE_180)

    #to calculate the distance
    distance_sum = 0;
    for i in range(0,3):
        distance_sum = distance_sum + distance()
    
    avg_distance = distance_sum / 3
    
    if counter == 0:
        pwm.ChangeDutyCycle(duty)
        duty -= rate
        if duty <= 3.5:
            counter = 1
            time.sleep(1)
    if counter == 1:
        pwm.ChangeDutyCycle(duty)
        duty += rate
        if duty >= 6.25:
            counter = 0
            time.sleep(1)
    
    cv2.putText(image, 'Duty :'+str(duty)+'%', (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(image, "Distance : "+str(avg_distance)+"cm", (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    out.write(image)
    
    # show the frame to our screen
    cv2.imshow("Frame", image)
    
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # press the 'q' key to stop the video stream
    if key == ord("q"):
        pwm.stop()
        GPIO.cleanup()
        break
    
#cv2.waitKey(0)
#cv2.destroyAllWindows()