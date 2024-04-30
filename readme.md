# Robo-code
This uses micropython for the ev3 bricks!

# IMPORTANT THINGS
 - See below about the sim
 - Positions of the objects are stored in `dat/positions.json` :)
## Installation
### Must install
`pip install pybricks numpy`

### For the sim
`pip install pygame pyttsx3`

And also `pip install simpleaudio`, but if that doesn't work then you won't be able to have the sounds in the sim, but that's OK

If running on Ubuntu you MUST install tkinter with `sudo apt-get install -y python3-tk`, and the others with `sudo apt install libespeak1` and `sudo apt-get install -y libasound2-dev` for the sim to work.

And on Windows make sure you ticked the `Tcl support for Python 3.11` when installing python, or it will error with `Tkinter does not exist` (Just try the code first and if it doesn't work then worry about this)

# Simulator
 - Example code (for *my* way of working on the project) is stored in the `examples/` directory, just copy all the files to the main directory to see it work! Also has the simulator in it too!
