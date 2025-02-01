import os
import pygame
import platform
import time

def init_pygame():
    """Initialize pygame with platform-specific settings"""
    print("Initializing pygame...")
    
    if platform.system() == 'Darwin':
        print("Running on macOS")
        os.environ['SDL_VIDEODRIVER'] = 'cocoa'
        os.environ['SDL_THREADSAFE'] = '1'
        os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
    
    try:
        pygame.init()
        pygame.display.init()
        print("Pygame initialized successfully")
        return True
    except Exception as e:
        print(f"Error initializing pygame: {str(e)}")
        return False 