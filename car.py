# York Technical College
# EET 273 - Electronics Senior Project
# Raspberry Pi RC Car with automatic headlights and collision avoidance
# Authors:
# Alex Hester
# Ali Abdur-Rashid
# Jerry Britt

import RPi.GPIO as GPIO
import time
import subprocess
import pygame
from pygame.constants import JOYBUTTONDOWN, JOYBUTTONUP, JOYAXISMOTION, JOYHATMOTION, JOYDEVICEADDED

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(8, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(10, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(12, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

GPIO.setup(16, GPIO.OUT)
GPIO.output(16,GPIO.LOW)

GPIO.setup(18, GPIO.OUT)
GPIO.output(18,GPIO.LOW)

GPIO.setup(32, GPIO.OUT)
driveMotorForward = GPIO.PWM(32,1000)
driveMotorForward.start(0)

GPIO.setup(36, GPIO.OUT)
driveMotorReverse = GPIO.PWM(36,1000)
driveMotorReverse.start(0)

GPIO.setup(38, GPIO.OUT)
steerMotorRight = GPIO.PWM(38, 1000)
steerMotorRight.start(0)

GPIO.setup(40, GPIO.OUT)
steerMotorLeft = GPIO.PWM(40, 1000)
steerMotorLeft.start(0)

def flashLights(_range = 3, _time = .5):
    for i in range(_range):
        GPIO.output(16,GPIO.HIGH)
        GPIO.output(18,GPIO.HIGH)
        time.sleep(_time)
        GPIO.output(16,GPIO.LOW)
        GPIO.output(18,GPIO.LOW)
        time.sleep(_time)

def steering(steerAmount):
    # Normalize Duty Cycle
    if steerAmount < -0.05:
        direction = "LEFT"
        dutyCycle = steerAmount * -50
    elif steerAmount > 0.05:
        direction = "RIGHT"
        dutyCycle = steerAmount * 50
    else:
        direction = "NONE"
        dutyCycle = 0
    if dutyCycle > 50: dutyCycle = 50
    elif dutyCycle < 0: dutyCycle = 0
    print(dutyCycle)
    # Apply to motor
    if direction == "LEFT":
        steerMotorLeft.ChangeDutyCycle(dutyCycle)
        steerMotorRight.ChangeDutyCycle(0)
    elif direction == "RIGHT":
        steerMotorRight.ChangeDutyCycle(dutyCycle)
        steerMotorLeft.ChangeDutyCycle(0)
    else:
        steerMotorLeft.ChangeDutyCycle(0)
        steerMotorRight.ChangeDutyCycle(0)

driveDutyCycle = 0

def drive(driveAmount, direction):
    # Normalize Duty Cycle
    global driveDutyCycle
    driveDutyCycle = ((driveAmount+1) / 2) * 100
    if driveDutyCycle > 100: driveDutyCycle = 100
    elif driveDutyCycle < 0: driveDutyCycle = 0

    # Apply to motor
    if direction == "FORWARD":
        driveMotorForward.ChangeDutyCycle(driveDutyCycle)
        driveMotorReverse.ChangeDutyCycle(0)
    elif direction == "REVERSE":
        driveMotorForward.ChangeDutyCycle(0)
        driveMotorReverse.ChangeDutyCycle(driveDutyCycle)
    else:
        driveMotorForward.ChangeDutyCycle(0)
        driveMotorReverse.ChangeDutyCycle(0)

def brake(loopTime):
    global driveDutyCycle
    if driveDutyCycle == 0:
        return
    print("Braking...")
    startTime = time.time()
    elapsedTime = 0
    while elapsedTime < loopTime:
        elapsedTime = time.time() - startTime
        driveMotorForward.ChangeDutyCycle(100)
        driveMotorReverse.ChangeDutyCycle(100)
    driveMotorForward.ChangeDutyCycle(0)
    driveMotorReverse.ChangeDutyCycle(0)

def waitForController():
    pygame.init()
    pygame.joystick.init()
    while True:
        for event in pygame.event.get():
            if event.type == JOYDEVICEADDED:
                gamepad = pygame.joystick.Joystick(event.device_index)
                print(gamepad.get_name() + " Connected!")
                return gamepad
        flashLights(1)
        print("Waiting for controller")

gamepad = waitForController()

lightOverride=False
pressedLB = False
pressedRB = False
pressedA = False
pressedB = False
canRunForward = True
canRunReverse = True
frontSensorLast = True
rearSensorLast = True

while True or KeyboardInterrupt:
#RearSensor
    if GPIO.input(10) == False and pressedA == False:
        print("Rear Obstacle Detected")
        if rearSensorLast == True:
            brake(.5)
            rearSensorLast = False
        canRunReverse = False
        time.sleep(.05)
    else:
        canRunReverse = True
        rearSensorLast = True

#FrontSensor
    if GPIO.input(8) == False and pressedA == False:
        print("Front Obstacle Detected")
        if frontSensorLast == True:
            brake(.5)
            frontSensorLast = False
        canRunForward = False
        time.sleep(.05)
    else:
        canRunForward = True
        frontSensorLast = True

# LightSensor
    if lightOverride == False:
        if GPIO.input(12) == False:
            GPIO.output(16,GPIO.LOW)
            GPIO.output(18,GPIO.LOW)
            time.sleep(.05)
        else:
            GPIO.output(16,GPIO.HIGH)
            GPIO.output(18,GPIO.HIGH)
            time.sleep(.05)

    if pressedA and pressedLB and pressedRB:
        print("Shutting down...")
        flashLights(10, .1)
        subprocess.run("sudo shutdown now", shell = True)
        exit()

# Braking
    if pressedB:
        driveMotorForward.ChangeDutyCycle(100)
        driveMotorReverse.ChangeDutyCycle(100)


    for event in pygame.event.get():

# Buttons
        if event.type == JOYBUTTONDOWN:
            if event.button == 0: # A
                pressedA = True
            elif event.button == 1: # B
                pressedB = True
            elif event.button == 6: # LB
                pressedLB = True
            elif event.button == 7: # RB
                pressedRB = True

        elif event.type == JOYBUTTONUP:
            if event.button == 0: # A
                pressedA = False
            elif event.button == 1: # B
                pressedB = False
            elif event.button == 6: # LB
                pressedLB = False
            elif event.button == 7: # RB
                pressedRB = False

# Triggers
        elif event.type == JOYAXISMOTION:
    # Left Stick (Steering)
            if event.axis == 0:
                steering(event.value)

    # RIGHT TRIGGER (FORWARD)
            elif event.axis == 4 and canRunForward == True:
                drive(event.value, "FORWARD")

    # LEFT TRIGGER (REVERSE)
            elif event.axis == 5 and canRunReverse == True:
                drive(event.value, "REVERSE")
    # D-PAD
        elif event.type == JOYHATMOTION:
            if event.value == (-1,0):
                lightOverride = not lightOverride
            elif event.value == (1,0):
                if GPIO.input(16) == 1:
                    GPIO.output(16,GPIO.LOW)
                    GPIO.output(18,GPIO.LOW)
                else:
                    GPIO.output(16,GPIO.HIGH)
                    GPIO.output(18,GPIO.HIGH)
                time.sleep(.05)
                