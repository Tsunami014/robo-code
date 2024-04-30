import math, pygame
import numpy as np

def scale_sur(sur, size, verbose=False) -> pygame.Surface | tuple[pygame.Surface, bool, float, pygame.Surface, float]:
    scaled = pygame.Surface(size)
    scaled.fill(0)
    scale = min([size[0] / sur.get_width(), size[1] / sur.get_height()])
    newsur = pygame.transform.scale(sur, (int(sur.get_width() * scale), int(sur.get_height() * scale)))
    if newsur.get_width() < size[0]:
        diff = (size[0] - newsur.get_width()) / 2
        scaled.blit(newsur, (diff, 0))
    else:
        diff = (size[1] - newsur.get_height()) / 2
        scaled.blit(newsur, (0, diff))
    if verbose:
        return scaled, newsur.get_width() < size[0], diff, newsur, scale
    return scaled

def rotate(origin, point, angle): # Thanks, https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python !
    """
    Rotate a point clockwise by a given angle around a given origin.

    The angle should be given in degrees.
    """
    angle = math.radians(-angle)
    ox, oy = origin
    px, py = point
    
    cos = math.cos(angle)
    sin = math.sin(angle)
    
    ydiff = (py - oy)
    xdiff = (px - ox)

    qx = ox + cos * xdiff - sin * ydiff
    qy = oy + sin * xdiff + cos * ydiff
    return qx, qy

def fixangle(angle):
    angle = angle % 360
    if angle > 180:
        angle = (-180) + (angle - 180)
    return angle

def toPolar(origin, point): # Thanks to https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates
    x, y = point[0] - origin[0], point[1] - origin[1]
    rho = np.sqrt(x**2 + y**2)
    phi = fixangle(180 - math.degrees(np.arctan2(y, x)))
    return (rho, phi) # rho: distance, phi: angle
