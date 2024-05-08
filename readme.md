# Robo-code
This uses micropython for the ev3 bricks!

# IMPORTANT THINGS
 - See below about the sim
 - Positions of the objects are stored in `dat/positions.json` :)
## Installation
### Must install
`pip install pybricks`

### For the sim
`pip install pygame pyttsx3 numpy shapely`

And also `pip install simpleaudio`, but if that doesn't work then you won't be able to have the sounds in the sim, but that's OK

If running on Ubuntu you MUST install tkinter with `sudo apt-get install -y python3-tk`, and the others with `sudo apt install libespeak1` and `sudo apt-get install -y libasound2-dev` for the sim to work.

And on Windows make sure you ticked the `Tcl support for Python 3.11` when installing python, or it will error with `Tkinter does not exist` (Just try the code first and if it doesn't work then worry about this)

# Simulator
 - Example code (for *my* way of working on the project) is stored in the `examples/` directory, just copy all the files to the main directory to see it work! Also has the simulator in it too!

## TODOS FOR THE SIM
Sorted in order of most to least important, tho the sim still works; these are just room for improvement.
 - [ ] Fix collisions
 - [ ] Add rotation to the collisions
 - [ ] Fix the still says what buttons are pressed when in path plotter mode
 - [ ] Make you able to manually specify a point to modify coords of
 - [ ] Make you able to see exactly where to put the robot to start
 - [ ] Fix the text for the buttons in path plotter mode going off the edge of the screen
 - [ ] Able to resize anything
