from main import Robot
from sim import EV3BrickSim, DriveBaseSim

r = Robot()
r.ev3 = EV3BrickSim()
r.driving = DriveBaseSim(*r.drivingconfig)
r.ev3.simulate(r.test_song, r.driving)