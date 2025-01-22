import asyncio
import random
import logging
import pygame
import re

from llm.output_model import Command
from providers.io_provider import IOProvider

from typing import List

class TextWindow:

    def __init__(self):
        self.messages: list[str] = []

        self.io_provider = IOProvider()

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

    def get_earliest_time(self) -> float:
        earliest_time = float("inf")
        for value in self.io_provider.inputs.values():
            timestamp = value.timestamp
            if timestamp < earliest_time:
                earliest_time = timestamp
        return earliest_time

    def run(self, commands: List[Command]) -> None:

        earliest_time = self.get_earliest_time()

        # make the background white
        self.display_surface.fill(self.white)

        y = 15
        for action, values in self.io_provider.inputs.items():
            inp = f"{(values.timestamp - earliest_time):.3f}:: {action} :: {values.input}"
            self.text = self.font.render(inp, True, self.black, self.white)
            self.textRect = self.text.get_rect()
            self.textRect.topleft = (20, y)
            y += 20
            self.display_surface.blit(self.text, self.textRect)

        dft = f"{self.io_provider.fuser_end_time - earliest_time:.3f}"
        self.text = self.font.render(f"Fuse_time: {dft}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        dst = f"{float(self.io_provider.llm_start_time) - earliest_time:.3f}"
        self.text = self.font.render(f"LLM_begin: {dst}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        processing_time_s = float(self.io_provider.llm_end_time) - float(self.io_provider.llm_start_time)
        dt = f"{processing_time_s:.3f}"
        self.text = self.font.render(f"LLM_proc time: {dt}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        dtt = f"{float(self.io_provider.llm_end_time) - earliest_time:.3f}"
        self.text = self.font.render(f"LLM_done: {dtt}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.display_surface.blit(self.text, self.textRect)

        for command in commands:
            action_type = command.name
            action_spec = command.arguments[0].value
            action = action_type + "::" + action_spec
            self.text = self.font.render(action, True, self.black, self.white)
            self.textRect = self.text.get_rect()
            self.textRect.topleft = (20, y)
            y += 20
            self.display_surface.blit(self.text, self.textRect)

        pygame.display.update()