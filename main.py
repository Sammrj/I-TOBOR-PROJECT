#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor as Motor
from pybricks.parameters import Port
from pybricks.parameters import Direction
from pybricks.tools import wait

# Create your objects here




# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize a motor at port B.
test_motor = Motor(Port.B)

#ev3.light.on(color)
#ev3.speaker.say("SAMUEL... SAMUEL, ")
test_motor.run(50000)

wait(30000)
wait(30000)


# Write your program here

# Play a sound.
ev3.speaker.beep("test")

# Run the motor up to 500 degrees per second. To a target angle of 90 degrees.

#ev3.speaker.beep()
#test_motor.run_target(5000, 30)

# Play another beep sound.
#ev3.speaker.beep(frequency=1000, duration=500)
