import asyncio
import random
import logging
import pygame
import re

from llm.output_model import LLM_full, Command
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

    def input_clean(self, input, earliest_time) -> str:
        st = input
        st = st.strip()
        st = st.replace("\n", "")
        st = re.sub(r"\s+", " ", st) # replace runs of whitespace
        st = st.replace("INPUT // START ", "")
        st = st.replace(" // END", "")

        sts = st.split("::")
        time = float(sts[0])
        time_rezero = time - earliest_time
        time_st = f"{time_rezero:.3f}::{sts[-1]}"

        return time_st

    def get_earliest_time(self, input_list: list[str]) -> float:
        times = []
        for input in input_list:
            times.append(float(input.split("::")[0]))
        return min(times)

    def print_raw(self, llm: LLM_full) -> None:
        logging.debug(f"SimText input: {llm.prompt}")
        logging.debug(f"SimText input list: {llm.input_list}")
        logging.debug(f"SimText input list: {llm.commands.commands}")

        earliest_time = self.get_earliest_time(llm.input_list)

        # make the background white
        self.display_surface.fill(self.white)

        # clean up inputs - remove formatting strings
        inputs = llm.input_list

        y = 15
        for input in llm.input_list:
            inp = self.input_clean(input, earliest_time)
            logging.info(f"SimText display: {inp}")
            self.text = self.font.render(inp, True, self.black, self.white)
            self.textRect = self.text.get_rect()
            self.textRect.topleft = (20, y)
            y += 20
            self.display_surface.blit(self.text, self.textRect)
        
        dft = f"{float(llm.time_fuse) - earliest_time:.3f}"
        self.text = self.font.render(f"Fuse_time: {dft}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        dst = f"{float(llm.time_submit) - earliest_time:.3f}"
        self.text = self.font.render(f"LLM_begin: {dst}", True, self.black, self.white)
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

        dtt = f"{float(llm.time_done) - earliest_time:.3f}"
        self.text = self.font.render(f"LLM_done: {dtt}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        for command in llm.commands.commands:
            action_type = command.name
            action_spec = command.arguments[0].value
            action = action_type + "::" + action_spec
            self.text = self.font.render(action, True, self.black, self.white)
            self.textRect = self.text.get_rect()
            self.textRect.topleft = (20, y)
            y += 20
            self.display_surface.blit(self.text, self.textRect)

        pygame.display.update()
