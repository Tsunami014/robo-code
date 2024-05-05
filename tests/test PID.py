import pygame
pygame.init()

from sim.motors import Control, Stop
from sim.time import set_fr, tick

win = pygame.display.set_mode((500, 500))
fr = 4
set_fr(fr)

PID = Control()
PID.pid(0.1)
quality = 2
current_spd = 0
current_pos = 250
goal = 250
prevs = [0, 250]

side_size = (250 - 4, 500)
left = pygame.Surface(side_size)
right = pygame.Surface(side_size)

clock = pygame.time.Clock()
run = True
while run:
    tick()
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            run = False
    if pygame.mouse.get_pressed()[0]:
        goal = 500 - pygame.mouse.get_pos()[1]
    
    current_spd = PID(goal - current_pos, Stop.COAST, goal != current_pos)
    nleft = pygame.Surface(side_size)
    nleft.blit(left, (-quality, 0))
    pygame.draw.line(nleft, (50, 100, 255), (side_size[0]-quality, 250-prevs[0]), (side_size[0], 250-current_spd), 4)
    prevs[0] = current_spd
    left = nleft
    
    nright = pygame.Surface(side_size)
    nright.blit(right, (-quality, 0))
    current_pos = current_pos + current_spd
    pygame.draw.line(nright, (255, 100, 50), (side_size[0]-quality, 500-prevs[1]), (side_size[0], 500-current_pos), 4)
    prevs[1] = current_pos
    right = nright
    
    win.fill((255, 255, 255))
    win.blit(left, (0, 0))
    win.blit(right, (250 + 4, 0))
    pygame.draw.line(win, (255, 0, 0), (0, 500-goal), (500, 500-goal), 2)
    pygame.draw.line(win, 0, (250, 0), (250, 500), 8)
    pygame.display.update()
    clock.tick(fr)
