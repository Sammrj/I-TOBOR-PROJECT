#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor as Motor
from pybricks.ev3devices import TouchSensor as TouchSensor
from pybricks.parameters import Port
from pybricks.parameters import Direction
from pybricks.tools import wait

# Initialize the EV3 Brick.
ev3 = EV3Brick()
#touch = TouchSensor(Port.A)
test_motor = Motor(Port.B)
ev3.speaker.say ("Hello... I'm TOBOR :  T, O, B, O, R,")
ev3.speaker.say ("Please, turn my motor")
i=0
while i<5:
    speed = test_motor.speed() # recuperation de la vitesse de Tobor
    if speed !=0: 
        wait(1000) # attendre 1 minute
        test_motor.stop()  # Arreter le moteur
        ev3.speaker.say("angle... {} {}".format(int (test_motor.angle()),"degree")) # Tobor dit l'angle actuel de son moteur
        test_motor.run_target(5000,0) # Repositionner le moteur à zero dégré
        wait(1000) # attendre une minnute
        test_motor.stop()
        ev3.speaker.beep() # faire biper la box
        ev3.speaker.say ("again") # TOBOR veut qu'on tourne à nouveau son moteur
        i+=1 

"""
# Create your objects here

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize a motor at port B.
test_motor = Motor(Port.B)

#ev3.light.on(color)
#ev3.speaker.say("SAMUEL... SAMUEL, ")
test_motor.run(50000)

wait(30000)

# Write your program here

# Play a sound.
#ev3.speaker.beep()

# Run the motor up to 500 degrees per second. To a target angle of 90 degrees.

#ev3.speaker.beep()
#test_motor.run_target(5000, 30)

# Play another beep sound.
#ev3.speaker.beep(frequency=1000, duration=500)
"""