import os
import pygame
import platform

def init_pygame():
    if platform.system() == 'Darwin':  # macOS
        os.environ['SDL_VIDEODRIVER'] = 'dummy'
    pygame.init()
    return True 