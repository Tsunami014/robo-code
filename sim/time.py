FRAME = 0
FRAMERATE = 0

def tick():
    global FRAME
    FRAME += 1

def reset():
    global FRAME
    FRAME = 0

def set_fr(fr):
    global FRAMERATE
    FRAMERATE = fr
