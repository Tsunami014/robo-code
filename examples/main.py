#!/usr/bin/env pybricks-micropython
# We're robot #6!
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class Robot:
    def __init__(self):
        self.ev3 = EV3Brick()
        self.drivingconfig = (Motor(Port.B), Motor(Port.C), 55.5, 104)
        self.driving = DriveBase(*self.drivingconfig)
    
    def test_driving_simple(self):
        self.ev3.speaker.beep(duration=1000)
        while True:
            for i in range(4):
                self.driving.straight(100)
                self.ev3.speaker.beep(200*((i*2)+1))
                self.driving.turn(90)
                self.ev3.speaker.beep(200*((i*2)+2))
    
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
