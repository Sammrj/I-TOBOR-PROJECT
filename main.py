#!/usr/bin/env python3
 
 
from time import sleep
from smbus import SMBus
from pixycamev3.pixy2 import Pixy2
from ev3dev2.display import Display
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor import INPUT_1
from ev3dev2.port import LegoPort
from ev3dev2.sound import Sound
from sys import stderr
from Robot import Robot
from Robot import Robot
import multiprocessing 
from time import sleep
from serveur import ExchangeWithUser
import threading


# Initalize Class
robot = Robot()
global state_of_the_procedure 
state_of_the_procedure = 'start'
mode_calibration = True

exc = ExchangeWithUser()  # class à initialiser dans le code de Tobor


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

def path_1():
    # Snack track
    while True:
        angle = 90
        robot.move_forward()
        robot.turn_right(angle)
        if robot.isPathOver("snack_track"):
            # state_of_the_procedure
            print("path is over",file=stderr)
            s1[1] = 'go_home'
        robot.move_target_forward(150)
        robot.turn_right(angle*2)
        robot.move_forward()
        robot.turn_left(angle)
        if robot.isPathOver("snack_track"):
            # state_of_the_procedure
            print("path is over",file=stderr)
            s1[1] = 'go_home'
        robot.move_target_forward(150)
        robot.turn_left(0)
    return

def test():
    
    print("start",file=stderr)
    while not exc.connected :
        print(exc.connected,file=stderr)
    while not exc.myThread.send_rec_data.getDataRec:
        pass
    print("debut",file=stderr)
    exc.myThread.send_rec_data.reset_previous_data_()    
    exc.message_to_send("?")
    while exc.myThread.send_rec_data.getDataRec is None : 
        pass
    print("Tobor a recu"+exc.myThread.send_rec_data.getDataRec,file=stderr)  



def path_2():
    # efficient track

    robot.turn_right(45)
    robot.move_target_forward(400)
    robot.turn_right(405)
    sleep(1.0)
    if s1[1] != 'go_to_target' :
        # state_of_the_procedure
        robot.turn_left(0)
        print("path is over",file=stderr)
        s1[1] = 'go_home'
    return

def path_3():
    # snail track
    angle = 90
    distance = 400
    while True:
        robot.move_target_forward(distance)
        robot.turn_right(angle)
        robot.move_target_forward(distance)
        robot.turn_right(angle)
        robot.move_target_forward(distance)
        robot.turn_right(angle)
        if robot.isPathOver("snail_track", distance):
            # state_of_the_procedure
            print("path is over",file=stderr)
            s1[1] = 'go_home'
        angle = angle + 90
        distance = distance - (distance*(20/100))
    return

def go_to_target():
        robot.stop()
        sleep(1);
        # spkr.speak("go to target")
        while robot.get_us_value()>8:
            current_angle = robot.get_angle_value()
            x = s1[0][8]
            y = s1[0][10]
            robot.stop()
            if sigs == s1[0][7]*256 + s1[0][6]:
                # target detected, control motors
                print("x : "+str(x),file=stderr)
                if x > 140 and  x < 150:
                    print("inside",file=stderr)
                    robot.move_forward()
                elif x <= 140:
                    print("right",file=stderr)
                    print("current_angle : "+str(current_angle),file=stderr)
                    robot.turn_right_with_precision()
                elif x >= 150 :
                    print("left",file=stderr)
                    robot.turn_left_with_precision()
                    
            else:
                # target not detected, stop motors
                robot.stop()
            
        robot.stop()
        s1[1] = "catch_target"
        return

def catch_target():
    robot.catch_target();
    s1[1] = 'go_home'
def go_home():
        
    robot.turn_right(180)
    robot.move_forward()
    robot.turn_right(270)
    robot.move_forward()
    return

def find_target():
    first_time = True
def find_target():
    first_time = True
    while True:
        # Clear display
        lcd.clear()
        lcd.clear()
        # Request block
        
        bus.write_i2c_block_data(address, 0, data)
        # Read block
        block = bus.read_i2c_block_data(address, 0, 20)
        s1[0] = block
        # print(block,file=stderr)
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
        # lcd.draw.rectangle((xa, ya, xb, yb), fill='black')
        # Update display to how rectangle
        # lcd.update()
        
        # print("s1[0][8]"+str(s1[0][8]),file=stderr)
        # print("y"+str(block[10]),file=stderr)
        
        if block[7] == 0 and first_time:
            # condition à réalisé une seule fois
            first_time = False
            s1[1] = 'go_to_target'

        # print(block[7],file=stderr)
    return

# MAIN
if mode_calibration == True:

    print(robot.get_us_value(),file = stderr )
    th1 = threading.Thread(target=exc.launch)
    th1.start()
    test()
    # robot.turn_right_with_precision()
    # robot.turn_right_with_precision()
    # robot.turn_right_with_precision()
    # robot.catch_target()
    # robot.drop_target()
    # angle = 90
    # robot.move_target_forward(150)
    # robot.turn_right(angle)
    # robot.move_target_forward(150)
    # robot.turn_right(angle*2)
    # robot.move_target_forward(150)
    # robot.turn_right(angle*3)
    # robot.move_target_forward(150)
    # robot.turn_right(angle*4)
    # angle = -90
    # robot.move_target_forward(150)
    # robot.turn_left(angle)
    # robot.move_target_forward(150)
    # robot.turn_left(angle*2)
    # robot.move_target_forward(150)
    # robot.turn_left(angle*3)
    # robot.move_target_forward(150)
    # robot.turn_left(angle*4)
else:
    spkr.speak('Start')  

    # Lancement de la procédure
    

    thread1 = multiprocessing.Process(target=find_target)
    thread1.start()
    thread2 = multiprocessing.Process(target=path_2)
    th3 = multiprocessing.Process(target=exc.launch)
    th3.start()  # lancement de l'échange

    while True:
        if s1[1] == "start" :
            print("start",file=stderr)
            s1[1] = 'waiting'
            thread2 = multiprocessing.Process(target=path_2)
            thread2.start()
        elif s1[1] == "go_to_target":
            print("FIND",file=stderr)
            s1[1] = 'waiting'
            thread2.terminate()
            thread2 = multiprocessing.Process(target=go_to_target)
            thread2.start()
        elif s1[1] == "catch_target":
            print("CATCH",file=stderr)
            s1[1] = 'waiting'
            thread2.terminate()
            thread2 = multiprocessing.Process(target=catch_target)
            thread2.start() 
        elif s1[1] == "go_home":
            print("HOME",file=stderr)
            s1[1] = 'waiting'
            thread2.terminate()
            thread2 = multiprocessing.Process(target=go_home )
            thread2.start()
    spkr.speak('End')