import pygame.draw
import math

def collideLineLine(l1_p1, l1_p2, l2_p1, l2_p2): # Thanks to https://stackoverflow.com/questions/64095396/detecting-collisions-between-polygons-and-rectangles-in-pygame !
    # normalized direction of the lines and start of the lines
    P  = pygame.math.Vector2(*l1_p1)
    line1_vec = pygame.math.Vector2(*l1_p2) - P
    R = line1_vec.normalize()
    Q  = pygame.math.Vector2(*l2_p1)
    line2_vec = pygame.math.Vector2(*l2_p2) - Q
    S = line2_vec.normalize()

    # normal vectors to the lines
    RNV = pygame.math.Vector2(R[1], -R[0])
    SNV = pygame.math.Vector2(S[1], -S[0])
    RdotSVN = R.dot(SNV)
    if RdotSVN == 0:
        return False

    # distance to the intersection point
    QP  = Q - P
    t = QP.dot(SNV) / RdotSVN
    u = QP.dot(RNV) / RdotSVN

    return t > 0 and u > 0 and t*t < line1_vec.magnitude_squared() and u*u < line2_vec.magnitude_squared() and (line_intersection((l1_p1, l1_p2), (l2_p1, l2_p2)) is not None)

def closest_line(point, lines):
    dist = -1
    closest = None
    ps = lines.copy() + [lines[0]]
    for i in range(len(ps)-1):
        dist_x = ((ps[i][0] + ps[i+1][0]) / 2) - point[0]
        dist_y = ((ps[i][1] + ps[i+1][1]) / 2) - point[1]
        full_dist = math.sqrt(abs(dist_x) ** 2 + abs(dist_y) ** 2)
        if dist == -1 or (full_dist < dist):
            dist = full_dist
            closest = [ps[i], ps[i+1]]
    return closest

def line_intersection(line1, line2): # Thanks to https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines !
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

class Obj:
    def __init__(self, colour, points):
        self.colour = colour
        self.points = points
        self.centre = self.find_centre()
    
    def find_centre(self, points=None):
        if points is None:
            points = self.points
        return (sum([i[0] for i in points]) / len(points), sum([i[1] for i in points]) / len(points))
    
    def moveby(self, x, y):
        self.points = [(i[0]+x, i[1]+y) for i in self.points]
        self.centre = self.find_centre()
    
    def is_in_rect(self, points, win=None):
        points_centre = self.find_centre(points)
        if points == [] or self.points == []:
            return False
        closest_point_line = closest_line(self.centre, points)
        closest_this_line = closest_line(points_centre, self.points)
        edgecollide = collideLineLine(*closest_point_line, *closest_this_line)
        if edgecollide:
            pass
            # intersection = line_intersection(closest_point_line, closest_this_line)
        if win is not None:
            pygame.draw.line(win, 0, self.find_centre(closest_point_line), self.find_centre(closest_this_line), 8)
            prev = points[0]
            for i in points[1:]:
                pygame.draw.line(win, self.colour, prev, i, 10)
                prev = i
            pygame.draw.line(win, self.colour, self.points[-1], self.points[0], 10)
        return edgecollide or any(
            collideLineLine(self.points[i], self.points[len(self.points) // 2 + i], *closest_point_line) for i in range(len(self.points)//2)
        ) or (abs(points_centre[0] - self.centre[0]) + abs(points_centre[1] - self.centre[1])) < min([
            abs(points_centre[0] - i[0]) + abs(points_centre[1] - i[1]) for i in self.points
        ])
    
    def draw(self, win, points=[]):
        if len(self.points) < 2:
            return
        if points != []:
            self.is_in_rect(points, win)
        prev = self.points[0]
        for i in self.points[1:]:
            pygame.draw.line(win, self.colour, prev, i, 10)
            prev = i
        pygame.draw.line(win, self.colour, self.points[-1], self.points[0], 10)
        pygame.draw.circle(win, self.colour, self.centre, 10)
