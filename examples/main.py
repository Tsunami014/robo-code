#!/usr/bin/env pybricks-micropython
# We're robot #6!
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class Sensors:
    Ultrasonic = UltrasonicSensor(Port.S4)

class Robot:
    def __init__(self):
        self.ev3 = EV3Brick()
        self.driving = DriveBase(Motor(Port.B), Motor(Port.C), wheel_diameter=55.5, axle_track=104)
    
    def test_follow(self):
        while True:
            self.ev3.speaker.beep()
            if Sensors.Ultrasonic.distance() > 30:
                self.driving.straight(30)
            else:
                self.driving.straight(-30)
    
    def test_song(self):
        d = {
            Button.LEFT:  (100, ImageFile.LEFT),
            Button.UP:    (300, ImageFile.UP),
            Button.RIGHT: (500, ImageFile.RIGHT),
            Button.DOWN:  (700, ImageFile.DOWN)
        }
        while True:
            #self.ev3.screen.clear()
            for i in self.ev3.buttons.pressed():
                if i in d:
                    self.ev3.speaker.beep(d[i][0])
                    self.ev3.screen.load_image(d[i][1])
                    break

if __name__ == '__main__':
    robot = Robot()
    robot.test_song()
