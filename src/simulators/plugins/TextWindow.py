import asyncio
import random
import logging
import pygame

from llm.output_model import LLM_full
from typing import Dict, Optional

class TextWindow:

    def __init__(self):
        self.messages: list[str] = []
        self.name = __class__
        pygame.init()
 
        # define the RGB value for white,
        #  green, blue colour .
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 128)
        self.black = (0, 0, 0)
         
        # assigning values to X and Y variable
        self.X = 800
        self.Y = 400
 
        # create the display surface object
        # of specific dimension..e(X, Y).
        self.display_surface = pygame.display.set_mode((self.X, self.Y))
         
        # set the pygame window name
        pygame.display.set_caption('DogText')
 
        self.font = pygame.font.Font('freesansbold.ttf', 14)

    def input_clean(self, input) -> str:
        st = input
        st = st.strip()
        st = st.replace("INPUT\n        // START\n        ", "")
        st = st.replace("::", " ")
        st = st.replace("\n        // END", "")
        return st

    def print_raw(self, llm: LLM_full) -> None:
        logging.info(f"SimText input: {llm.prompt}")
        logging.info(f"SimText input list: {llm.input_list}")

        # make the background white
        self.display_surface.fill(self.white)

        # clean up inputs - remove formatting strings
        inputs = llm.input_list

        y = 15
        for input in llm.input_list:
            inp = self.input_clean(input)
            logging.info(f"SimText display: {inp}")
            self.text = self.font.render(inp, True, self.black, self.white)
            self.textRect = self.text.get_rect()
            # self.textRect.left = (20, y)
            self.textRect.topleft = (20, y)
            y += 20
            self.display_surface.blit(self.text, self.textRect)
        
        self.text = self.font.render(f"LLM_begin: {llm.time_submit}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        self.text = self.font.render(f"LLM_done: {llm.time_done}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        processing_time_s = float(llm.time_done) - float(llm.time_submit)
        dt = f"{processing_time_s:.3f}"
        self.text = self.font.render(f"LLM_proc time: {dt}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        pygame.display.update()





