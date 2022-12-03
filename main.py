#!/usr/bin/env python3
 
from time import sleep
from smbus import SMBus
from pixycamev3.pixy2 import Pixy2
from ev3dev2.display import Display
from ev3dev2.sensor import INPUT_1
from ev3dev2.port import LegoPort
from ev3dev2.sound import Sound
from sys import stderr
from Robot import Robot
import multiprocessing 

# Initalize Class
robot = Robot()
global state_of_the_procedure 
state_of_the_procedure = 'start'
mode_calibration = True 
# initialization of the pixy
pixy2 = Pixy2(port=1, i2c_address=0x54)
in1 = LegoPort(INPUT_1)
in1.mode = 'other-i2c'
sleep(0.5)
bus = SMBus(3)
address = 0x54
sigs = 1
data = [174, 193, 32, 2, sigs, 1]


# information from the system
lcd = Display()
spkr = Sound()

# variable accesible from all thread
manager = multiprocessing.Manager()
s1 = manager.list()
s1.append(0)
s1.append(state_of_the_procedure)

def path():
    while True:
        target = 90
        robot.move_forward()
        robot.turn_right(target)
        if robot.isPathOver():
            # state_of_the_procedure
            s1[1] = 'go_home'
        robot.move_target_forward()
        robot.turn_right(target*2)
        robot.move_forward()
        robot.turn_left(target)
        if robot.isPathOver():
            # state_of_the_procedure
            s1[1] = 'go_home'
        robot.move_target_forward()
        robot.turn_left(0)
    return

def find_target():
    first_time = True
    while True:
        # Clear display
        lcd.clear()
        # Request block
        
        bus.write_i2c_block_data(address, 0, data)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        s1[0] = block
        # print(block,file=stderr)
        # Extract data
        sig = block[7]*256 + block[6]
        x = block[9]*256 + block[8]
        y = block[11]*256 + block[10]
        w = block[13]*256 + block[12]
        h = block[15]*256 + block[14]
        # Scale to resolution of EV3 display:
        # Resolution Pixy2 while color tracking; (316x208)
        # Resolution EV3 display: (178x128)
        x *= 0.6
        y *= 0.6
        w *= 0.6
        h *= 0.6
        # Calculate rectangle to draw on display
        dx = int(w/2)
        dy = int(h/2)
        xa = x - dx
        ya = y + dy
        xb = x + dx
        yb = y - dy
        # Draw rectangle on display
        lcd.draw.rectangle((xa, ya, xb, yb), fill='black')
        # Update display to how rectangle
        lcd.update()
        # print("x"+str(block[8]),file=stderr)
        # print("y"+str(block[10]),file=stderr)
        
        if block[7] == 0 and first_time:
            # condition à réalisé une seule fois
            first_time = False
            s1[1] = 'go_to_target'
            spkr.speak('Target find')

        print(block[7],file=stderr)
    return

# MAIN
if mode_calibration == True:
    robot.catch_target()
    # target = 90
    # robot.move_target_forward()
    # robot.turn_right(target)
    # robot.move_target_forward()
    # robot.turn_right(target*2)
    # robot.move_target_forward()
    # robot.turn_right(target*3)
    # robot.move_target_forward()
    # robot.turn_right(target*4)
    
    # target = -90
    # robot.move_target_forward()
    # robot.turn_left(target)
    # robot.move_target_forward()
    # robot.turn_left(target*2)
    # robot.move_target_forward()
    # robot.turn_left(target*3)
    # robot.move_target_forward()
    # robot.turn_left(target*4)
else:
    spkr.speak('Start')  

    # Lancement de la procédure
    thread1 = multiprocessing.Process(target=find_target)
    thread1.start()
    thread2 = multiprocessing.Process(target=path)
    while True:
        if s1[1] == "start" :
            print("start",file=stderr)

            s1[1] = 'waiting'
            thread2 = multiprocessing.Process(target=path)
            thread2.start()
        elif s1[1] == "go_to_target":
            print("FIND",file=stderr)
            s1[1] = 'waiting'
            thread2.terminate()
            thread2 = multiprocessing.Process(target=robot.go_to_target)
            thread2.start()
        elif s1[1] == "catch_target":
            print("CATCH",file=stderr)
            s1[1] = 'waiting'
            thread2.terminate()
            thread2 = multiprocessing.Process(target=robot.catch_target)
            thread2.start()
        elif s1[1] == "go_Home":
            print("HOME",file=stderr)
            s1[1] = 'waiting'
            thread2.terminate()
            thread2 = multiprocessing.Process(target=robot.go_home)
            thread2.start()

    spkr.speak('End')
