#!/usr/bin/env python3
from ev3dev2.sensor import INPUT_2,INPUT_3,INPUT_4
from ev3dev2.sensor.lego import InfraredSensor,GyroSensor,UltrasonicSensor
from ev3dev2.motor import OUTPUT_C,OUTPUT_B,MoveDifferential,LargeMotor, Motor, OUTPUT_A
from ev3dev2.wheel import EV3Tire
from Odometry import Odometry
from sys import stderr
from math import sin, cos, pi
class Robot:
    
    # Motor
    global tank_drive 
    global D_WHEEL_MM  
    global arm_motor
    D_WHEEL_MM = 102
    tank_drive = MoveDifferential(OUTPUT_B, OUTPUT_C,EV3Tire,D_WHEEL_MM)
    arm_motor = Motor(OUTPUT_A)

    # Odopmetry
    global odo
    global right_motor 
    global left_motor 
    global perimetre
    global position
    global compensation_right
    
    right_motor = Motor(OUTPUT_B)
    left_motor = Motor(OUTPUT_C)
    odo = Odometry(right_motor);
    perimetre = 17.3
    position = [0,0]
    compensation_right = 0

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
    # def __init__(self):
    #     self.compensation_right = 50

    global angletest
    angletest = 0;        

    def left_rotation(self,angle):
        left_motor.on_for_degrees(25,angle)
        return

    def right_rotation(self,angle):
        right_motor.on_for_degrees(25,angle)
        return
    
    def __init__(self):
        "Initialization"
        self.odo = Odometry(right_motor)
        self.angle = 0
        self.tank_drive = MoveDifferential(OUTPUT_B, OUTPUT_C,EV3Tire,D_WHEEL_MM)

    def get_angle_value(self):
        return tank_drive.gyro.angle

    def get_ir_value(self):
        return ir.value()

    def get_us_value(self):
        distance = us.value()/10
        print(distance,file = stderr)
        return distance

    def turn_right_with_precision(self):
        # odometry
        right_degree_at_start = right_motor.position *(360 / right_motor.count_per_rot)
        # -----------
        tank_drive.turn_right(-10,5, brake=True, block=True)
        # odometry
        right_degree_at_end = right_motor.position *(360 / right_motor.count_per_rot)
        # odo.update_compensation(right_degree_at_start,right_degree_at_end)
        # -----------
        return
    
    def turn_left_with_precision(self):
        # odometry
        right_degree_at_start = right_motor.position *(360 / right_motor.count_per_rot)
        # -----------

        tank_drive.turn_left(-10,5, brake=True, block=True)
        
        # odometry
        right_degree_at_end = right_motor.position *(360 / right_motor.count_per_rot)
        # odo.update_compensation(right_degree_at_start,right_degree_at_end)
        # -----------
        
        return

    def move_forward_with_us_sensor(self):
        odo.update_odometry(tank_drive.gyro.angle)
        tank_drive.off()
        distance = us.value()/10
        tank_drive.on(25,25)
        while distance > 30:
            distance = us.value()/10
            # print("ir distance : "+str(distance),file=stderr)
            # sprint(s1[0], file=stderr) 
        tank_drive.off()
        odo.update_odometry(tank_drive.gyro.angle)
        return
    
    def move_forward(self):
        odo.update_odometry(tank_drive.gyro.angle)
        tank_drive.off()
        distance = ir.value()
        tank_drive.on(25,25)
        while distance > 30:
            distance = ir.value()
            # print("ir distance : "+str(distance),file=stderr)
            # sprint(s1[0], file=stderr) 
        tank_drive.off()
        odo.update_odometry(tank_drive.gyro.angle)
        return

    def move_target_forward(self,target,brake = True):
        odo.update_odometry(tank_drive.gyro.angle)
        tank_drive.off()
        distance = ir.value()
        if distance >= 20 :
            tank_drive.on_for_distance(25, target, brake=True, block=True)
        odo.update_odometry(tank_drive.gyro.angle)
        return
    
    def move_target_forward_with_us_sensor(self,target):
        odo.update_odometry(tank_drive.gyro.angle)
        tank_drive.off()
        distance = us.value()/10
        if distance >= 4 :
            tank_drive.on_for_distance(25, target, brake=False, block=True)
        odo.update_odometry(tank_drive.gyro.angle)
        return

    def stop(self):
        tank_drive.off()
        odo.update_odometry(tank_drive.gyro.angle)
        return

    def turn_right(self,target):
        global angletest
        # ----------
        right_degree_at_start = right_motor.position *(360 / right_motor.count_per_rot)
        # -----------
        angle1 = tank_drive.gyro.angle
        if target > angle1 :
            tank_drive.on(-7,7)
            print("angle de depart"+str(tank_drive.gyro.angle),file=stderr)
            print("target : "+str(target),file=stderr)
            while tank_drive.gyro.angle < target :
                angle1 = tank_drive.gyro.angle
                print(angle1,file=stderr)
            tank_drive.off()
            angle2 = tank_drive.gyro.angle
            angletest = tank_drive.gyro.angle
            print( "valeur angle"+str(angletest),file=stderr)
            # gestion de la positon des roues
            right_degree_at_end = right_motor.position *(360 / right_motor.count_per_rot)
            # calcul de compensation
            odo.update_compensation(right_degree_at_start,right_degree_at_end)
            # fin
            print("right_degree_at_end : "+str(right_degree_at_end),file=stderr)
            print("right_degree_at_start : "+str(right_degree_at_start),file=stderr)
            print("angle d'arret ?: "+str(angle1),file=stderr)
            print("angle finale ? : "+str(angle2),file=stderr)
            print("decalage : " +str((angle2 - angle1)),file=stderr)        
            
        return

    def turn_left(self,target):
        global angletest
        #  
        right_degree_at_start = right_motor.position *(360 / right_motor.count_per_rot)
        # fin
        angle1 = tank_drive.gyro.angle
        if target < angle1 :
            print("angle de depart"+str(tank_drive.gyro.angle),file=stderr)
            print("target : "+str(target),file=stderr)
            tank_drive.on(7,-7)
            
            while tank_drive.gyro.angle > target :
                angle1 = tank_drive.gyro.angle
                print(angle1,file=stderr)
                pass
            angletest = tank_drive.gyro.angle
            print( "valeur angle"+str(angletest),file=stderr)
            tank_drive.off()
            angle2 = tank_drive.gyro.angle
            print("target : "+str(target),file=stderr)
            print("angle d'arret ?: "+str(angle1),file=stderr)
            print("angle finale ? : "+str(angle2),file=stderr)
            print("decalage : " +str((angle2 - angle1)),file=stderr)

            # gestion de la positon des roues
            right_degree_at_end = right_motor.position *(360 / right_motor.count_per_rot)
            # calcul de compensation
            odo.update_compensation(right_degree_at_start,right_degree_at_end)
            # fin
                    
            # tank_drive.turn_to_angle(25, -90)
        return
    
    def catch_target(self):
        print("catch the target ",file=stderr)
        arm_motor.on_for_degrees(10,80)
        return
    
    def drop_target(self):
        print("catch the target ",file=stderr)
        arm_motor.on_for_degrees(10,-60)
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
            if distance < 150:
                print("distance ispathover : "+str(distance),file=stderr )
                return True
            else:
                return False

    def go_home(self):
        angle = tank_drive.gyro.angle
        print("angle initial : "+str(angle),file=stderr)
        isBetween = True
        i = 0
        while isBetween :
            if 0+i*360 <= angle <360 + i*360 :      
                print("inside")  
                print("i = "+str(i))
                self.turn_right(180+i*360)
                self.move_forward()
                self.turn_right(270 +i*360)
                self.move_forward()
                isBetween = False
            elif 0 -i*360 > angle >= -360 - i*360 :
                print("inside2")
                print("i = "+str(i))
                self.turn_right(-180-i*360)
                self.move_forward()
                self.turn_right(-270 -i*360)
                self.move_forward()
                isBetween = False
            else:
                print("print i"+str(i))
                i = i + 1
        return

                
                
