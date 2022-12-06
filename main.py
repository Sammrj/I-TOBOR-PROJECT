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

# Connect LargeMotors
rmotor = LargeMotor(OUTPUT_B)
lmotor = LargeMotor(OUTPUT_C)

# Defining constants
X_REF = 158  # X-coordinate of referencepoint
Y_REF = 150  # Y-coordinate of referencepoint
KP = 0.4     # Proportional constant PID-controller
KI = 0.01    # Integral constant PID-controller
KD = 0.05    # Derivative constant PID-controller
GAIN = 5    # Gain for motorspeed

# Initializing PID variables
integral_x = 0
derivative_x = 0
last_dx = 0
integral_y = 0
derivative_y = 0
last_dy = 0

#Function name : move_forward
#description : function to make Tobor move forward
#output : 
def move_forward():
    
    tank_drive.off()
    distance = ir.value()
    tank_drive.on(-25,-25)
    while distance > 30 and not s1[0]:
        distance = ir.value()
        # print(s1[0], file=stderr)
    tank_drive.off()
    return

#Function name : move_target_forward
#description : function to make Tobor move in front of the target
#output : 
def move_target_forward():
    tank_drive.off()
    distance = ir.value()
    tank_drive.on_for_distance(-25, 150, brake=True, block=True)
    return

# def move_backward():
#     spkr.speak('Right')
#     tank_drive.on_for_degrees(25, 25, 230, brake=True, block=False)
#     return

#Function name : turn_right
#description : function to make Tobor turn to the right
#input : 
#output :
def turn_right(target):
    # spkr.speak('Right')
    
    motor2.on(-25)
    print("angle de depart"+str(tank_drive.gyro.angle),file=stderr)
    print("target : "+str(target),file=stderr)
    while tank_drive.gyro.angle < target :
        angle1 = tank_drive.gyro.angle
        print(angle1,file=stderr)
    motor2.off()
    angle2 = tank_drive.gyro.angle
    print("angle d'arret ?: "+str(angle1),file=stderr)
    print("angle finale ? : "+str(angle2),file=stderr)
    print("decalage : " +str((angle2 - angle1)),file=stderr)        
    # tank_drive.turn_to_angle(25, -90)
    return

#Function name : turn_left
#description : function to make Tobor turn to the left
#input : 
#output :
def turn_left(target):
    
    # spkr.speak('Right')
    print("angle de depart"+str(tank_drive.gyro.angle),file=stderr)
    print("target : "+str(target),file=stderr)
    motor1.on(-25)
    while tank_drive.gyro.angle > target :
        angle1 = tank_drive.gyro.angle
        print(angle1,file=stderr)
        pass
    motor1.off()
    angle2 = tank_drive.gyro.angle
    print("target : "+str(target),file=stderr)
    print("angle d'arret ?: "+str(angle1),file=stderr)
    print("angle finale ? : "+str(angle2),file=stderr)
    print("decalage : " +str((angle2 - angle1)),file=stderr)
            
    # tank_drive.turn_to_angle(25, -90)
    return

#Function name : isPathfinished
#description : function to check the state of the path of Tobor 
#output :
def isPathfinished():
    distance = ir.value()
    print(distance, file=stderr)
    if distance > 40:
        return False
    else:
        return True

#Function name : initialisation
#description : function to initialize the robot 
#output :
def initialisation():
    # spkr.speak('Initialisation')
    # Retourne la position initial du robot parmis les 4 possibilités

    frontValue = ir.value()
    turn_right()
    rightValue = ir.value()
    # console.text_at("front : %03d" % (frontValue), column=3, row=5, reset_console=False, inverse=True)
    # console.text_at("front : %03d" % (rightValue), column=3, row=8, reset_console=False, inverse=True)

    # while True:
    #     frontValue = ir.value()
    #     print(frontValue)
    
    if frontValue >= 40 and rightValue >= 40:
        spkr.speak('libre et libre')
        turn_left()
    elif frontValue >= 40 and rightValue < 40:
        spkr.speak('libre et bloque')
        turn_left()
        turn_left()
    elif frontValue < 40 and rightValue < 40:
        spkr.speak('Bloque et bloque')
        turn_right()
    else :
        spkr.speak('Bloque et libre')
    return

#Function name : limit_speed
#description : function to limit the speed of Tobor 
#input :
#output :
def limit_speed(speed):
    """ Limit speed in range [-1000,1000] """
    if speed > 1000:
        speed = 1000
    elif speed < -1000:
        speed = -1000
    return speed

#Function name : test
#description : function to test the angle of turn of Tobor 
#output :
def test():
    # spkr.speak('Right')
    motor2.on(-25)
    while tank_drive.gyro.angle < 89 :
        angle1 = tank_drive.gyro.angle
        print(angle1,file=stderr)
        pass
    motor2.off()
    angle2 = tank_drive.gyro.angle
    print("target : "+str(target),file=stderr)
    print("angle d'arret : "+str(angle1),file=stderr)
    print("angle finale : "+str(angle2),file=stderr)
    print("decalage : " +str(abs(angle2 - angle1)),file=stderr)
    # tank_drive.turn_to_angle(25, -90)
    return

s1.append(state_of_the_procedure)

#Function name : path
#description : function to define the type of path the robot will do 
#output :
def path():
    
    # initialisation()
    tank_drive.odometry_start()

    integral_x = 0
    derivative_x = 0
    last_dx = 0
    integral_y = 0
    derivative_y = 0
    last_dy = 0
    
    multiplicateur = 1
    target = 90
    targetDroit = 90
    targetGauche = 90

    while not s1[1] and not s1[0]: 
        
        if not s1[0]:
            tank_drive.off()
            distance = ir.value()
            tank_drive.on(-25,-25)
            while distance > 30 and not s1[0]:
                distance = ir.value()
                # print(s1[0], file=stderr)
            tank_drive.off()
        if not s1[0]:
            motor2.on(-25)
            while tank_drive.gyro.angle < target :
                angle1 = tank_drive.gyro.angle
                print(angle1,file=stderr)
                pass
            motor2.off()
            angle2 = tank_drive.gyro.angle
            print("target : "+str(target),file=stderr)
            print("angle d'arret ?: "+str(angle1),file=stderr)
            print("angle finale ? : "+str(angle2),file=stderr)
            print("decalage : " +str((angle2 - angle1)),file=stderr)        
            # tank_drive.turn_left(25, -90, brake=True, block=True, error_margin=1, use_gyro=False)
        
        if  isPathfinished() and not s1[0]:
            s1[1] = True
            
            
        if not s1[1] and not s1[0]:
            tank_drive.off()
            distance = ir.value()
            tank_drive.on_for_distance(-25, 150, brake=True, block=True)
        if not s1[0] and not s1[1]:
            motor2.on(-25)
            while tank_drive.gyro.angle < target*2 :
                angle1 = tank_drive.gyro.angle
                print(angle1,file=stderr)
                pass
            motor2.off()
            angle2 = tank_drive.gyro.angle
            print("target : "+str(target),file=stderr)
            print("angle d'arret ?: "+str(angle1),file=stderr)
            print("angle finale ? : "+str(angle2),file=stderr)
            print("decalage : " +str((angle2 - angle1)),file=stderr) 
            # tank_drive.turn_left(25, -90, brake=True, block=True, error_margin=1, use_gyro=False)
        if not s1[0] and not s1[1]:
            tank_drive.off()
            distance = ir.value()
            tank_drive.on(-25,-25)
            while distance > 30 and not s1[0]:
                distance = ir.value()
                # print(s1[0], file=stderr)
            tank_drive.off()
        if not s1[0] and not s1[1]:
            motor1.on(-25)
            while tank_drive.gyro.angle > target :
                angle1 = tank_drive.gyro.angle
                print(angle1,file=stderr)
                pass
            motor1.off()
            angle2 = tank_drive.gyro.angle
            print("target : "+str(target),file=stderr)
            print("angle d'arret ?: "+str(angle1),file=stderr)
            print("angle finale ? : "+str(angle2),file=stderr)
            print("decalage : " +str((angle2 - angle1)),file=stderr)
            # tank_drive.turn_right(25, -90, brake=True, block=True, error_margin=1, use_gyro=False)

        if isPathfinished() and not s1[0]:
            s1[1] = True
            

        if not s1[1] and not s1[0]:
            tank_drive.off()
            distance = ir.value()
            tank_drive.on_for_distance(-25, 150, brake=True, block=True)
        if not s1[0] and not s1[1]:
            motor1.on(-25)
            while tank_drive.gyro.angle > 0 :
                angle1 = tank_drive.gyro.angle
                print(angle1,file=stderr)
                pass
            motor1.off()
            angle2 = tank_drive.gyro.angle
            print("target : "+str(target),file=stderr)
            print("angle d'arret ?: "+str(angle1),file=stderr)
            print("angle finale ? : "+str(angle2),file=stderr)
            print("decalage : " +str((angle2 - angle1)),file=stderr)
            # tank_drive.turn_right(25, -90, brake=True, block=True, error_margin=1, use_gyro=False) 

    
    if s1[0] and not s1[2]:
        spkr.speak("go to target")
        inside = False
        while ir.value()>10:
            current_angle = tank_drive.gyro.angle
            # print("infrarouge:"+str(ir.value()),file=stderr)
            # print("pixy:"+str(s1[3]),file=stderr)
            x = s1[3][8]
            y = s1[3][10]
            tank_drive.off()
            if sigs == s1[3][7]*256 + s1[3][6]:
                # SIG1 detected, control motors
                print("x : "+str(x),file=stderr)
                # print("y : "+str(y),file=stderr)
                if x > 135 and  x < 165:
                    print("inside",file=stderr)
                    inside = True
                    tank_drive.on(-25,-25)
                elif x <= 135:
                    inside = False
                    print("right",file=stderr)
                    print("current_angle : "+str(current_angle),file=stderr)
                    motor1.on_for_degrees(-25,10)
                elif x >= 165 :
                    print("left",file=stderr)
                    inside = False
                    motor2.on_for_degrees(-25,10)
            else:
                # SIG1 not detected, stop motors
                rmotor.stop()
                lmotor.stop()
                # last_dx = 0
                # last_dy = 0
                # integral_x = 0
                # integral_y = 0
        rmotor.stop()
        lmotor.stop()

#Function name : run_time_motors
#description : function to define the movement of the 2 motors on the robot  
#input :
#output :
def run_time_motors(motor1,motor2):
    motor1.run_target(500,-1000,then=Stop.BRAKE,wait=False)
    motor2.run_target(500,-1000,then=Stop.BRAKE,wait=True)
    ev3.speaker.beep()
    wait(1000)
    motor1.run_target(500,0,then=Stop.BRAKE,wait=False)
    motor2.run_target(500,0,then=Stop.BRAKE,wait=True)
    ev3.speaker.beep()
    return True

    if s1[0] or  s1[1]:
        spkr.speak('go home')
        turn_right(180)
        move_forward()
        turn_right(270)
        move_forward()
    return

#Function name : findTarget
#description : function to find the target with the pixy camera on the robot  
#output :
def findTarget():

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

#Function name : goHome
#description : function to make the robot go Home after finishing his path 
#output :
def goHome():
    spkr.speak('go home')
    turn_right(180)
    move_forward()
    turn_right(270)
    move_forward()
    print(block[7],file=stderr)
    return

# MAIN
#description : main script of the Software 
#output :
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
