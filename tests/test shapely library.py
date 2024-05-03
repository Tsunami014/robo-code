# time: ~20-25 mins 

import pygame, shapely.geometry, math
pygame.init()

win = pygame.display.set_mode((900, 900))

shapes = [
    
[[164, 720], [152, 765], [212, 785], [220, 742]], [[164, 720], [152, 765], [212, 785], [220, 742]]
]
colours = [(255, 0, 0), (0, 0, 255)]

font = pygame.font.SysFont(None, 64)

holding = None
run = True
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            run = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            for s in range(len(shapes)):
                if holding is None:
                    for corner in range(len(shapes[s])):
                        dist_x = abs(event.pos[0] - shapes[s][corner][0])
                        dist_y = abs(event.pos[1] - shapes[s][corner][1])
                        dist = math.sqrt(dist_x ** 2 + dist_y ** 2)
                        if dist <= 10:
                            holding = [s, corner]
                            break
        elif event.type == pygame.MOUSEBUTTONUP:
            holding = None
    if holding is not None and pygame.mouse.get_pressed()[0]:
        shapes[holding[0]][holding[1]] = pygame.mouse.get_pos()
    win.fill((255, 255, 255))
    for s in shapes:
        prev = s[0]
        for edge in s[1:]:
            pygame.draw.line(win, colours[shapes.index(s)], prev, edge, 8)
            prev = edge
        pygame.draw.line(win, colours[shapes.index(s)], prev, s[0], 8)
        for corner in s:
            pygame.draw.circle(win, 0, corner, 10)
    shap_1 = shapely.geometry.Polygon(shapes[0])
    shap_2 = shapely.geometry.Polygon(shapes[1])
    line_1 = shapely.geometry.LineString(shapes[1][:2])
    pygame.draw.line(win, (0, 255, 0), shapes[1][0], shapes[1][1], 8)
    win.blit(font.render(f'Intersects: {shap_1.intersects(shap_2)}, Intersects: {line_1.intersects(shap_1)}', 1, 0), (0, 0))
    pygame.display.update()
