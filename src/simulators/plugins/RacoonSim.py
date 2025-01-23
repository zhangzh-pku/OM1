import asyncio
import random
import logging
import pygame, gif_pygame
import re, os

from llm.output_model import LLM_full, Command
from typing import Dict, Optional

class RacoonSim:

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
        self.lightblue = (84,118,156)
         
        self.X = 800
        self.Y = 630

        self.clock = pygame.Clock()
 
        self.display = pygame.display.set_mode((self.X, self.Y))
        pygame.display.set_caption('Racoon Simulator')
        self.display.fill(self.lightblue)

        self.surface_text = pygame.Surface((800, 230))
        self.surface_text.fill(self.white)

        self.surface_ani = pygame.Surface((400, 400))
        self.surface_ani.fill(self.lightblue)

        self.font = pygame.font.Font('freesansbold.ttf', 14)
        self.path = os.path.join(os.path.dirname(__file__), 'assets')

        self.action_walk = gif_pygame.load(os.path.join(self.path, "walk.gif"))
        self.action_run = gif_pygame.load(os.path.join(self.path, "run.gif"))
        self.action_idle = gif_pygame.load(os.path.join(self.path, "idle.gif"))
        self.action_sit = gif_pygame.load(os.path.join(self.path, "crouch.gif"))

        self.a_s = ""

    def _tick(self) -> None:

        self.surface_ani.fill(self.lightblue)

        if self.a_s == "walk":
            self.action_walk.render(self.surface_ani, (0, 0))
        elif self.a_s == "run":
            self.action_run.render(self.surface_ani, (0, 0))
        elif self.a_s == "sit":
            self.action_sit.render(self.surface_ani, (0, 0))
        else:
            self.action_idle.render(self.surface_ani, (0, 0))

        self.display.blit(self.surface_ani, (180, 230))

        # this is what updates everything
        pygame.display.flip()
        
    # async def _run_animation_loop(self) -> None:
    #     while True:
    #         await asyncio.sleep(0.1)
    #         await self._tick()

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
            if input is not None and "::" in input:
                times.append(float(input.split("::")[0]))
        return min(times)

    def update(self, llm: LLM_full) -> None:
        
        logging.debug(f"SimText input: {llm.prompt}")
        logging.debug(f"SimText input list: {llm.input_list}")
        logging.debug(f"SimText input list: {llm.commands.commands}")

        earliest_time = self.get_earliest_time(llm.input_list)

        # make the background white
        self.surface_text.fill(self.white)

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
            self.surface_text.blit(self.text, self.textRect)
        
        dft = f"{float(llm.time_fuse) - earliest_time:.3f}"
        self.text = self.font.render(f"Fuse_time: {dft}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.surface_text.blit(self.text, self.textRect)

        dst = f"{float(llm.time_submit) - earliest_time:.3f}"
        self.text = self.font.render(f"LLM_begin: {dst}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.surface_text.blit(self.text, self.textRect)

        processing_time_s = float(llm.time_done) - float(llm.time_submit)
        dt = f"{processing_time_s:.3f}"
        self.text = self.font.render(f"LLM_proc time: {dt}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.surface_text.blit(self.text, self.textRect)

        dtt = f"{float(llm.time_done) - earliest_time:.3f}"
        self.text = self.font.render(f"LLM_done: {dtt}", True, self.black, self.white)
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (20, y)
        y += 20
        self.surface_text.blit(self.text, self.textRect)

        for command in llm.commands.commands:
            action_type = command.name
            action_spec = command.arguments[0].value
            if action_type == "move":
                self.a_s = action_spec
            action = action_type + "::" + action_spec
            self.text = self.font.render(action, True, self.black, self.white)
            self.textRect = self.text.get_rect()
            self.textRect.topleft = (20, y)
            y += 20
            self.surface_text.blit(self.text, self.textRect)

        self.display.blit(self.surface_text, (0, 0))
               
        # just for now - simple hack
        self._tick()
