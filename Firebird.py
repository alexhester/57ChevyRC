# York Technical College - 2024
# Written, modeled, and constructed by Alex Hester
# Raspberry Pi RC Car with automatic headlights and collision avoidance

import RPi.GPIO as GPIO
import time
import subprocess
import pygame
from pygame.constants import JOYBUTTONDOWN, JOYBUTTONUP, JOYAXISMOTION, JOYHATMOTION, JOYDEVICEADDED

maxSteerDC = 50
maxDriveDC = 100

lightSensor = 10
lights = [4, 7, 9, 17]
frontSensors = [8, 11]
rearSensors = [15, 27]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(lightSensor, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

for light in lights:
    GPIO.setup(light, GPIO.OUT)

for sensor in frontSensors:
    GPIO.setup(sensor, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

for sensor in rearSensors:
    GPIO.setup(sensor, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

GPIO.setup(23, GPIO.OUT)
driveMotorForward = GPIO.PWM(23,1000)
driveMotorForward.start(0)

GPIO.setup(18, GPIO.OUT)
driveMotorReverse = GPIO.PWM(18,1000)
driveMotorReverse.start(0)

GPIO.setup(25, GPIO.OUT)
steerMotorRight = GPIO.PWM(25, 1000)
steerMotorRight.start(0)

GPIO.setup(24, GPIO.OUT)
steerMotorLeft = GPIO.PWM(24, 1000)
steerMotorLeft.start(0)

def flashLights(_range = 3, _time = 0.5):
    for i in range(_range):
        for light in lights:
            GPIO.output(light, GPIO.HIGH)
        time.sleep(_time)
        for light in lights:
            GPIO.output(light, GPIO.LOW)
        time.sleep(_time)

def steer(steerAmount):
    global maxSteerDC
    # Normalize Duty Cycle
    if steerAmount < -0.05:
        direction = "LEFT"
        dutyCycle = steerAmount * -maxSteerDC
    elif steerAmount > 0.05:
        direction = "RIGHT"
        dutyCycle = steerAmount * maxSteerDC
    else:
        direction = "NONE"
        dutyCycle = 0
    if dutyCycle > maxSteerDC: dutyCycle = maxSteerDC
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
    global maxDriveDC
    global driveDutyCycle
    driveDutyCycle = ((driveAmount+1) / 2) * maxDriveDC
    if driveDutyCycle > maxDriveDC: driveDutyCycle = maxDriveDC
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
    global maxDriveDC
    global driveDutyCycle
    if driveDutyCycle == 0:
        return
    print("Braking...")
    startTime = time.time()
    elapsedTime = 0
    while elapsedTime < loopTime:
        elapsedTime = time.time() - startTime
        driveMotorForward.ChangeDutyCycle(maxDriveDC)
        driveMotorReverse.ChangeDutyCycle(maxDriveDC)
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

while KeyboardInterrupt:

    if pressedA == False:
        #RearSensor
        if GPIO.input(rearSensors[0]) == False or GPIO.input(rearSensors[1]) == False:
            print("Rear Obstacle Detected")
            if rearSensorLast == True:
                brake(0.5)
                rearSensorLast = False
            canRunReverse = False
            time.sleep(0.05)
        else:
            canRunReverse = True
            rearSensorLast = True

        #FrontSensor
        if GPIO.input(frontSensors[0]) == False or GPIO.input(frontSensors[1]) == False:
            print("Front Obstacle Detected")
            if frontSensorLast == True:
                brake(0.5)
                frontSensorLast = False
            canRunForward = False
            time.sleep(0.05)
        else:
            canRunForward = True
            frontSensorLast = True

    # LightSensor
    if lightOverride == False:
        if GPIO.input(lightSensor) == False:
            for light in lights:
                GPIO.output(light, GPIO.LOW)
            time.sleep(0.05)
        else:
            for light in lights:
                GPIO.output(light, GPIO.HIGH)
            time.sleep(0.05)

    # Shut Down
    if pressedA and pressedLB and pressedRB:
        print("Shutting down...")
        flashLights(10, .1)
        subprocess.run("sudo shutdown now", shell = True)
        exit()

    # Braking
    if pressedB:
        driveMotorForward.ChangeDutyCycle(maxDriveDC)
        driveMotorReverse.ChangeDutyCycle(maxDriveDC)

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
            if event.axis == 0: # Left Stick (Steering)
                steer(event.value)
            elif event.axis == 4 and canRunForward == True: # RIGHT TRIGGER (FORWARD)
                drive(event.value, "FORWARD")
            elif event.axis == 5 and canRunReverse == True: # LEFT TRIGGER (REVERSE)
                drive(event.value, "REVERSE")

        # D-PAD
        elif event.type == JOYHATMOTION:
            if event.value == (-1,0): # Right
                lightOverride = not lightOverride
            elif event.value == (1,0) and lightOverride == True: # Left
                if GPIO.input(lights[0]) == 1:
                    for light in lights:
                        GPIO.output(light, GPIO.LOW)
                else:
                    for light in lights:
                        GPIO.output(light, GPIO.HIGH)
                time.sleep(0.05)
