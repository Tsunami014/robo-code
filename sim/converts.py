from pygame.font import SysFont as PygameFont
from pygame.surface import Surface
from pygame.image import load
import pygame.draw

from pybricks.parameters import Color
from pybricks.media.ev3dev import Font, Image, ImageFile, SoundFile

import requests, io, simpleaudio

def ConvertColor(color: Color) -> tuple[int, int, int]:
    h = color.h
    s = color.s
    v = color.v
    if s:
        if h == 1.0: h = 0.0
        i = int(h*6.0); f = h*6.0 - i
        
        w = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))
        
        if i==0: return (v, t, w)
        if i==1: return (q, v, w)
        if i==2: return (w, v, t)
        if i==3: return (w, q, v)
        if i==4: return (t, w, v)
        if i==5: return (v, w, q)
    else: return (v, v, v)

def ConvertFont(font: Font) -> PygameFont:
    return PygameFont(font.family, font.height) # TODO: bold=font.style=='Bold'

# TODO: Return an image to say 'LOADING IMAGE' so it doesn't block the whole stream
IMAGECACHE = {}

def ConvertImageFile(imagefile: str | ImageFile) -> Surface:
    alls = [getattr(ImageFile,i) for i in dir(ImageFile) if i.upper() == i and i != '_BASE_PATH']
    if imagefile not in alls: raise ValueError('Invalid image')
    if imagefile in IMAGECACHE: return IMAGECACHE[imagefile]
    try:
        resp = requests.get('https://pybricks.com/ev3-micropython/_images'+imagefile[imagefile.rfind('/'):])
    except Exception as e:
        resp = e
        resp.status_code = -1
        resp.reason = str(e)
    if resp.status_code != 200:
        name = [i for i in dir(ImageFile) if i.upper() == i and i != '_BASE_PATH'][alls.index(imagefile)]
        sze = (160, 120)
        sur = Surface(sze)
        sur.fill((255, 255, 255))
        h = 10
        pygame.draw.rect(sur, (255, 35, 12), sur.get_rect(), 3, 2)
        subs = PygameFont(None, 20).render('Image failed to load!', 1, (255, 35, 12))
        sur.blit(subs, ((sze[0]-subs.get_width())/2, h))
        h += subs.get_height()+10
        subs = PygameFont('Comic Sans MS', 40).render(name, 1, (12, 112, 255))
        sur.blit(subs, ((sze[0]-subs.get_width())/2, h))
        h += subs.get_height()+10
        subs = PygameFont(None, 12).render('Reason: '+resp.reason, 1, (0, 0, 0))
        sur.blit(subs, ((sze[0]-subs.get_width())/2, h))
        IMAGECACHE[imagefile] = sur
        return sur
    
    data = io.BytesIO()
    for block in resp.iter_content(1024):
        if not block:
            break
        data.write(block)

    data.seek(0)
    s = load(data)
    IMAGECACHE[imagefile] = s
    return s

def ConvertImage(image: Image) -> Surface:
    pass # TODO: This

# TODO: Return a sound (maybe have the tts say it and have that a param of this func?) to say 'LOADING SOUND' so it doesn't block the whole stream
SOUNDCACHE = {}

def ConvertSoundFile(soundfile: str | SoundFile):
    if soundfile in SOUNDCACHE: return SOUNDCACHE[soundfile]
    audio_url = 'https://pybricks.com/ev3-micropython/_downloads'+soundfile[soundfile.rfind('/'):]
    try:
        resp = requests.get(audio_url)
    except Exception as e:
        resp = e
        resp.status_code = -1
        resp.reason = str(e)
    
    if resp.status_code != 200:
        # DO SOMTHING HERE
        return None
    
    content = resp.content
    wave = simpleaudio.WaveObject(audio_data=content,
                                sample_rate=22050,
                                num_channels=1,
                                bytes_per_sample=2)
    SOUNDCACHE[soundfile] = wave
    return wave
