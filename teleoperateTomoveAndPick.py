import RPi.GPIO as gpio
import time

    
def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)  #IN1
    gpio.setup(33, gpio.OUT)  #IN2
    gpio.setup(35, gpio.OUT)  #IN3
    gpio.setup(37, gpio.OUT)  #IN4

def gameover():
    # Set all pins low
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)


def forward(tf):
    init()
    # Left wheels
    gpio.output(31, True) 
    gpio.output(33, False) 
    # Right wheels
    gpio.output(35, False) 
    gpio.output(37, True) 
    # Wait
    time.sleep(tf)
    # Set all pins low and cleanup
    gameover()
    gpio.cleanup()


def reverse(tf):
    init()
    # Left wheels
    gpio.output(31, False) 
    gpio.output(33, True) 
    # Right wheels
    gpio.output(35, True) 
    gpio.output(37, False) 
    # Wait
    time.sleep(tf)
    # Set all pins low and cleanup
    gameover()
    gpio.cleanup()


def pivotright(tf):
    init()
    # Left wheels
    gpio.output(31, True)
    gpio.output(33, False)
    # Right wheels
    gpio.output(35, True)
    gpio.output(37, False)
    # Wait
    time.sleep(tf)
    # Set all pins low and cleanup
    gameover()
    gpio.cleanup()


def pivotleft(tf):
    init()
    # Left wheels
    gpio.output(31, False)
    gpio.output(33, True)
    # Right wheels
    gpio.output(35, False)
    gpio.output(37, True)
    # Wait
    time.sleep(tf)
    # Set all pins low and cleanup
    gameover()
    gpio.cleanup()


def key_input(event):
    init()
    print("Key: ", event)
    key_press = event
    tf = 1
    
    if key_press.lower() == 'w':
        forward(tf)
    elif key_press.lower() == 's':
        reverse(tf)
    elif key_press.lower() == 'a':
        pivotleft(tf)
    elif key_press.lower() == 'd':
        pivotright(tf)
    else:
        print("Invalid key pressed!")

if __name__ == '__main__':

    while True:
        key_press = input("Select driving mode: ")
        if key_press == 'q':
            break
        key_input(key_press)
