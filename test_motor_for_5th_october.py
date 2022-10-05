#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor as Motor
from pybricks.ev3devices import TouchSensor as TouchSensor
from pybricks.parameters import Port
from pybricks.parameters import Direction
from pybricks.parameters import Stop
from pybricks.tools import wait
#import threading

def run_time_motors(motor1,motor2):
    motor1.run_target(500,1000,then=Stop.BRAKE,wait=False)
    motor2.run_target(500,1000,then=Stop.BRAKE,wait=True)
    ev3.speaker.beep()
    wait(1000)
    motor1.run_target(500,0,then=Stop.BRAKE,wait=False)
    motor2.run_target(500,0,then=Stop.BRAKE,wait=True)
    ev3.speaker.beep()
    return True

test_touch = TouchSensor(Port.S2)
        
# Initialize the EV3 Brick.
ev3 = EV3Brick()
#touch = TouchSensor(Port.A)
test_motor1 = Motor(Port.B)
test_motor2 = Motor(Port.C)

test_motor1.run_target(1000,0,then=Stop.BRAKE,wait=True)
test_motor2.run_target(1000,0,then=Stop.BRAKE,wait=True)

ev3.speaker.say ("Hello... I'm TOBOR :  T, O, B, O, R,")
ev3.speaker.say ("Please, press my touch sensor")

i=0
motor_activates =False
while i<2:
    print("dans la boucle")
    if test_touch.pressed():
        if(not motor_activates):
            run_time_motors(test_motor1,test_motor2)
            motor_activates = True
            
    if( motor_activates and test_motor1.speed()==0 and test_motor2.speed()==0 ):
        motor_activates=False
        ev3.speaker.say ("Tobor is stopping...")
        #print("Tobor is stopping")
        i+=1  
  
#ev3.speaker.say ("Bye Bye")
