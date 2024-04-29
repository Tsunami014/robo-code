# TODO: Make this just an extension of test.py and use the don't show the code method like they do
from sim import EV3BrickSim, DriveBaseSim

ev3 = EV3BrickSim()
driving = DriveBaseSim()
ev3.simulate(lambda: None, driving, True) # Simulate nothing happening, but also make it in path plotter mode to make you able to edit stuff!
