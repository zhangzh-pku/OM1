import os
import pygame
import platform

def init_platform():
    """Initialize platform-specific settings"""
    if platform.system() == 'Darwin':
        os.environ['SDL_VIDEODRIVER'] = 'cocoa'
        os.environ['SDL_THREADSAFE'] = '1'
        os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
        os.environ['SDL_VIDEO_CENTERED'] = '1' 