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
    
    def collision(self, ps, angle, angle_dir, rotate_around, turn_collide):
        if isinstance(ps, (list, tuple)):
            points = ps
        elif isinstance(ps, Obj):
            points = ps.points
        else:
            raise TypeError(
                f'Inputted value for param \'ps\' "{ps}" has type {type(ps)} which is not a list, tuple or Obj!'
            )
        assert isinstance(turn_collide, int), f"Value for param 'turn_collide' \"{turn_collide}\" which has type {type(turn_collide)} which is not an int!"
        assert turn_collide in [0, 1], f"Value for param 'turn_collide' \"{turn_collide}\" is not 0 or 1!"
        if points == [] or self.points == []:
            return False
        point_box = shapely.geometry.Polygon(points)
        line = shapely.geometry.LineString(points[:2])
        dir = 1 if angle_dir > 1 else (-1 if angle_dir < -1 else 0)
        box = shapely.geometry.Polygon(self.points)
        attempts = -1 # Do not block anything
        if turn_collide == 1 and dir != 0:
            while (box.intersects(point_box) or line.intersects(box)) and attempts < 100:
                self.moveby(*rotate((rotate_around[0] - self.centre[0], rotate_around[1] - self.centre[1]), (0, 0), dir))
                box = shapely.geometry.Polygon(self.points)
                self.centre = self.find_centre()
                attempts += 1
        if attempts == 100 or attempts == -1:
            attempts = 0
            while (box.intersects(point_box) or line.intersects(box)) and attempts < 100:
                self.moveby(*rotate((0, 0), (0, 10), angle))
                box = shapely.geometry.Polygon(self.points)
                self.centre = self.find_centre()
                attempts += 1
    
    def update(self, win, objects, angle, angle_dir, rotate_around):
        if len(self.points) < 2:
            return
        for obj in objects:
            self.collision(obj[0], angle, round(angle_dir), rotate_around, obj[1])
        prev = self.points[0]
        for i in self.points[1:]:
            pygame.draw.line(win, self.colour, prev, i, 10)
            prev = i
        pygame.draw.line(win, self.colour, self.points[-1], self.points[0], 10)
        pygame.draw.circle(win, self.colour, self.centre, 10)
