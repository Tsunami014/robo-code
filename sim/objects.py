import pygame.draw
import shapely.geometry
from sim.mathMethods import rotate, toPolar

class Obj:
    def __init__(self, colour, points):
        self.colour = colour
        self.points = points
        self.orig_points = points.copy()
        self.centre = self.find_centre()
    
    def reset(self):
        self.points = self.orig_points.copy()
        self.centre = self.find_centre()
    
    def find_centre(self, points=None):
        if points is None:
            points = self.points
        return (sum([i[0] for i in points]) / len(points), sum([i[1] for i in points]) / len(points))
    
    def moveby(self, x, y):
        self.points = [(i[0]+x, i[1]+y) for i in self.points]
        self.centre = self.find_centre()
    
    def rotate(self, angle):
        self.points = [rotate(self.centre, i, angle) for i in self.points]
        self.centre = self.find_centre() # Though *should* be the same, but rounding errors occur so...
    
    def collision(self, points, angle):
        if points == [] or self.points == []:
            return False
        points = shapely.geometry.Polygon(points)
        box = shapely.geometry.Polygon(self.points)
        attempts = 0
        while box.intersects(points) and attempts < 100: # Do not block anything
            self.moveby(*rotate((0, 0), (0, 1), angle))
            box = shapely.geometry.Polygon(self.points)
            attempts += 1
    
    def update(self, win, objects, angle):
        if len(self.points) < 2:
            return
        for obj in objects:
            self.collision(obj, angle)
        prev = self.points[0]
        for i in self.points[1:]:
            pygame.draw.line(win, self.colour, prev, i, 10)
            prev = i
        pygame.draw.line(win, self.colour, self.points[-1], self.points[0], 10)
        pygame.draw.circle(win, self.colour, self.centre, 10)
