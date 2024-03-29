from pygame import Surface

from threading import Thread
from pygame.image import load
from pygame.font import SysFont as PygameFont
import requests, io, pygame.draw

IMAGECACHE = {}

class URLImage(Surface):
    def __init__(self, url):
        super().__init__((160, 120))
        self.fill((255, 255, 255))
        self.url = url
        self.sur = None
        self.t = Thread(target=self._get_im, daemon=True)
        self.t.start()
    
    def _get_im(self):
        try:
            resp = requests.get(self.url)
        except Exception as e:
            resp = e
            resp.status_code = -1
            resp.reason = str(e)
        if resp.status_code != 200:
            sze = (160, 120)
            sur = Surface(sze)
            sur.fill((255, 255, 255))
            h = 10
            pygame.draw.rect(sur, (255, 35, 12), sur.get_rect(), 3, 2)
            subs = PygameFont(None, 20).render('Image failed to load!', 1, (255, 35, 12))
            sur.blit(subs, ((sze[0]-subs.get_width())/2, h))
            h += subs.get_height()+10
            subs = PygameFont('Comic Sans MS', 40).render(self.name, 1, (12, 112, 255))
            sur.blit(subs, ((sze[0]-subs.get_width())/2, h))
            h += subs.get_height()+10
            subs = PygameFont(None, 12).render('Reason: '+resp.reason, 1, (0, 0, 0))
            sur.blit(subs, ((sze[0]-subs.get_width())/2, h))
            self.sur = sur
            return
        
        data = io.BytesIO()
        for block in resp.iter_content(1024):
            if not block:
                break
            data.write(block)

        data.seek(0)
        s = load(data)
        self.sur = s
    
    def get_sur(self):
        if self.sur is None:
            sze = (160, 120)
            sur = Surface(sze)
            sur.fill((255, 255, 255))
            pygame.draw.rect(sur, (255, 35, 12), sur.get_rect(), 3, 2)
            subs = PygameFont(None, 20).render('Image loading...', 1, (255, 35, 12))
            sur.blit(subs, ((sze[0]-subs.get_width())/2, 10))
            return sur
        else:
            return self.sur
