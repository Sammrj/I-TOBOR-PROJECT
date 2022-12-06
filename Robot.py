#!/usr/bin/env python3
from ev3dev2.sensor import INPUT_2,INPUT_3
from ev3dev2.sensor.lego import InfraredSensor,GyroSensor
from ev3dev2.motor import OUTPUT_C,OUTPUT_B,MoveDifferential,LargeMotor, Motor, OUTPUT_A
from ev3dev2.wheel import EV3Tire
from ev3dev2.console import Console
from ev3dev2.sensor import INPUT_1, INPUT_2,INPUT_3
from ev3dev2.sensor.lego import TouchSensor,InfraredSensor,GyroSensor
from ev3dev2.console import Console
from ev3dev2.wheel import EV3Tire
# librairie ev3dev equivalent
from ev3dev2.motor import OUTPUT_C,OUTPUT_B,MoveDifferential,LargeMotor
from ev3dev2.motor import Motor
from ev3dev2.motor import MoveTank
from sys import stderr
from threading import Thread
import sys
import time

class Robot:

    # Motor
    global tank_drive 
    global D_WHEEL_MM 
    global motor_left
    global motor_right 
    global arm_motor
    motor_left = Motor(OUTPUT_B)
    motor_right = Motor(OUTPUT_C)
    D_WHEEL_MM = 102
    tank_drive = MoveDifferential(OUTPUT_B, OUTPUT_C,EV3Tire,D_WHEEL_MM)
    arm_motor = Motor(OUTPUT_A)


    # Infrared
    global ir
    

    ir = InfraredSensor(INPUT_2)
    ir.mode = 'IR-PROX'
   
    # Gyroscope
    tank_drive.gyro = GyroSensor(INPUT_3)
    tank_drive.gyro.mode = 'GYRO-ANG'
    tank_drive.gyro.calibrate()
    tank_drive.gyro.reset()


    def __init__(self):
        "Initialization"
        
    def move_forward(self):
        
        tank_drive.off()
        distance = ir.value()
        tank_drive.on(25,25)
        while distance > 30:
            distance = ir.value()
            # print("ir distance : "+str(distance),file=stderr)
            # print(s1[0], file=stderr) 
        tank_drive.off()
        return

    def move_target_forward(self):
        tank_drive.off()
        distance = ir.value()
        if distance >= 20 :
            tank_drive.on_for_distance(25, 150, brake=True, block=True)
        return

    def stop(self):
        tank_drive.off()

    def turn_right(self,target):
        # spkr.speak('Right')
        
        tank_drive.on(25,-25)
        print("angle de depart"+str(tank_drive.gyro.angle),file=stderr)
        print("target : "+str(target),file=stderr)
        while tank_drive.gyro.angle < target :
            angle1 = tank_drive.gyro.angle
            print(angle1,file=stderr)
        tank_drive.off()
        angle2 = tank_drive.gyro.angle
        print("angle d'arret ?: "+str(angle1),file=stderr)
        print("angle finale ? : "+str(angle2),file=stderr)
        print("decalage : " +str((angle2 - angle1)),file=stderr)        
        # tank_drive.turn_to_angle(25, -90)
        return

    def turn_left(self,target):
        
        # spkr.speak('Right')
        print("angle de depart"+str(tank_drive.gyro.angle),file=stderr)
        print("target : "+str(target),file=stderr)
        tank_drive.on(-25,25)
        while tank_drive.gyro.angle > target :
            angle1 = tank_drive.gyro.angle
            print(angle1,file=stderr)
            pass
        tank_drive.off()
        angle2 = tank_drive.gyro.angle
        print("target : "+str(target),file=stderr)
        print("angle d'arret ?: "+str(angle1),file=stderr)
        print("angle finale ? : "+str(angle2),file=stderr)
        print("decalage : " +str((angle2 - angle1)),file=stderr)
                
        # tank_drive.turn_to_angle(25, -90)
        return
    
    def go_to_target(self):
        tank_drive.off()
        spkr.speak("go to target")
        while ir.value()>10:
            current_angle = tank_drive.gyro.angle
            x = s1[0][8]
            y = s1[0][10]
            tank_drive.off()
            if sigs == s1[0][7]*256 + s1[0][6]:
                # target detected, control motors
                print("x : "+str(x),file=stderr)
                if x > 135 and  x < 165:
                    print("inside",file=stderr)
                    tank_drive.on(-25,-25)
                elif x <= 135:
                    print("right",file=stderr)
                    print("current_angle : "+str(current_angle),file=stderr)
                    motor1.on_for_degrees(-25,10)
                elif x >= 165 :
                    print("left",file=stderr)
                    motor2.on_for_degrees(-25,10)
            else:
                # target not detected, stop motors
                motor_left.off()
                motor_right.off()
            
        motor_left.off()
        motor_right.off()
        s1[1] = "go_home"
        return

    def catch_target(self):
        print("catch the target ",file=stderr)
        arm_motor.on_for_degrees(10,30)
        return
    
    def go_home(self):
        spkr.speak('go home')
        turn_right(180)
        move_forward()
        turn_right(270)
        move_forward()
        return
    
    def isPathOver(self):
        distance = ir.value()
        print(distance, file=stderr)
        if distance > 40:
            return False
        else:
            return True
