from sys import stderr
from math import sin, cos, pi, radians
from ev3dev2.motor import OUTPUT_A

class Odometry:



    def __init__(self, rmotor):
        self.right_motor =  rmotor
        self.perimetre = 17.3
        self.position = [0,0]
        self.compensation_right = 0
        self.old_compensation = 0
        self.compensation = 0

    def           (self,angle = 0):
        
        angle = radians(angle)
        right_degree = (self.right_motor.position*(360 / self.right_motor.count_per_rot)) - self.compensation_right
        # left_degree = left_motor.position *(360 / left_motor.count_per_rot)
        distance = right_degree*self.perimetre/360
        
        x = cos(angle)*distance
        y = sin(angle)*distance
        self.position[0] += x
        self.position[1] += y
        # print(self.compensation_right,file=stderr)
        # print("cos(angle)"+str(cos(angle)),file=stderr)
        # print("sin(angle)"+str(sin(angle)),file=stderr)
        # print("angle : "+str(angle),file=stderr)
        # print("right : "+str(right_degree),file=stderr)
        # # print("left : "+str(left_motor.position *(360 / right_motor.count_per_rot)),file=stderr)
        # print("distance : "+str(distance),file = stderr)
        # print("position : "+str(self.position),file = stderr)
        
        return

    def update_compensation(self, oldvalue, newvalue):
        self.compensation = self.compensation + newvalue - oldvalue
        print("compensation"+str(self.compensation),file=stderr)
