#!/usr/bin/env python3
from ev3dev2.sensor import INPUT_2,INPUT_3,INPUT_4
from ev3dev2.sensor.lego import InfraredSensor,GyroSensor,UltrasonicSensor
from ev3dev2.motor import OUTPUT_C,OUTPUT_B,MoveDifferential,LargeMotor, Motor, OUTPUT_A
from ev3dev2.wheel import EV3Tire
from sys import stderr
 
class Robot:

    # Motor
    
    global tank_drive 
    global D_WHEEL_MM  
    global arm_motor
    D_WHEEL_MM = 102
    tank_drive = MoveDifferential(OUTPUT_B, OUTPUT_C,EV3Tire,D_WHEEL_MM)
    arm_motor = Motor(OUTPUT_A)


    # Infrared 1
    global ir
    ir = InfraredSensor(INPUT_2)
    ir.mode = 'IR-PROX'
    
    # US sensor 2
    global us
    us = UltrasonicSensor(INPUT_4)
    us.mode='US-DIST-CM'
    units = us.units

    # Gyroscope
    tank_drive.gyro = GyroSensor(INPUT_3)
    tank_drive.gyro.mode = 'GYRO-ANG'
    tank_drive.gyro.calibrate()
    tank_drive.gyro.reset()


    def __init__(self):
        "Initialization"

    def get_angle_value(self):
        return tank_drive.gyro.angle

    def get_ir_value(self):
        return ir.value()

    def get_us_value(self):
        distance = us.value()/10
        print(distance,file = stderr)
        return distance

    def turn_right_with_precision(self):
        tank_drive.turn_right(15,5, brake=True, block=True)
        return
    
    def turn_left_with_precision(self):
        tank_drive.turn_left(15,5, brake=True, block=True)
        return
    
    def move_forward(self):
        
        tank_drive.off()
        distance = ir.value()
        tank_drive.on(25,25)
        while distance > 30:
            distance = ir.value()
            # print("ir distance : "+str(distance),file=stderr)
            # sprint(s1[0], file=stderr) 
        tank_drive.off()
        return

    def move_target_forward(self,target):
        tank_drive.off()
        distance = ir.value()
        if distance >= 20 :
            tank_drive.on_for_distance(25, target, brake=True, block=True)
        return

    def stop(self):
        tank_drive.off()
        return

    def turn_right(self,target):
        # spkr.speak('Right')
        angle1 = tank_drive.gyro.angle
        if target > angle1 :
            tank_drive.on(-10,10)
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
        angle1 = tank_drive.gyro.angle
        if target < angle1 :
            # spkr.speak('Right')
            print("angle de depart"+str(tank_drive.gyro.angle),file=stderr)
            print("target : "+str(target),file=stderr)
            tank_drive.on(10,-10)
            
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
    
    def catch_target(self):
        print("catch the target ",file=stderr)
        arm_motor.on_for_degrees(10,60)
        return
    
    def drop_target(self):
        print("catch the target ",file=stderr)
        arm_motor.on_for_degrees(10,-60)
        return

    def go_home(self):
        
        turn_right(180)
        move_forward()
        turn_right(270)
        move_forward()
        return

    def isPathOver(self,string,distance = 0):
        if string == "snack_track":    
            distance = ir.value()
            print(distance, file=stderr)
            if distance > 40:
                return False
            else:
                return True
        elif string =="snail_track":
            if distance < 50:
                return True
            else:
                return False
            
            
