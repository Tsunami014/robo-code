import pygame.draw

class Obj:
    def __init__(self, colour, points):
        self.colour = colour
        self.points = points
    
    def moveby(self, x, y):
        self.points = [(i[0]+x, i[1]+y) for i in self.points]
    
    def draw(self, win):
        if len(self.points) < 2:
            return
        prev = self.points[0]
        for i in self.points[1:]:
            pygame.draw.line(win, self.colour, prev, i, 10)
            prev = i
        pygame.draw.line(win, self.colour, self.points[-1], self.points[0], 10)
