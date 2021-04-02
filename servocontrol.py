import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(36, GPIO.OUT)

pwm = GPIO.PWM(36, 50)
pwm.start(5)

rate = 0.05
duty = float(6.25)
while True:
    print(duty)
    pwm.ChangeDutyCycle(duty)
    duty -= rate
    time.sleep(0.2)
    if duty <= 3.5:
        break

time.sleep(1)

while True:
    print(duty)
    pwm.ChangeDutyCycle(duty)
    duty += rate
    time.sleep(0.2)
    if duty >= 6.25:
        break

pwm.stop()
GPIO.cleanup()
