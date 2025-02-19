import asyncio
import functools
import logging
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, List, Optional, TypeVar, Union

import openai

from llm import LLMConfig

from .io_provider import IOProvider

R = TypeVar("R")


@dataclass
class ChatMessage:
    role: str
    content: str


ACTION_MAP = {
    "emotion": "You felt: {}. ",
    "speak": "You said: {} ",
    "move": "You performed this motion: {}. ",
}


class LLMHistoryManager:
    def __init__(
        self,
        config: LLMConfig,
        client: Union[openai.AsyncClient, openai.OpenAI],
        system_prompt: str = "You are a helpful assistant that summarizes conversations accurately and concisely.",
        summary_prompt: str = "Please summarize the following conversation while preserving key information:\n\n",
    ):
        self.client = client

        # configuration
        self.config = config
        self.system_prompt = system_prompt
        self.summary_prompt = summary_prompt

        # task executor
        self._summary_task: Optional[asyncio.Task] = None

        # history buffer
        self.history: List[ChatMessage] = []

        # io provider
        self.io_provider = IOProvider()

    async def summarize_messages(self, messages: List[ChatMessage]) -> ChatMessage:
        """
        Summarize a list of messages using the OpenAI API.
        Returns a new message containing the summary.
        """
        try:
            for msg in messages:
                summary_prompt = self.summary_prompt + f"{msg.role}: {msg.content}\n"

            response = await self.client.chat.completions.create(
                model=self.config.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": summary_prompt},
                ],
            )

            summary = response.choices[0].message.content
            return ChatMessage(
                role="system", content=f"Previous conversation summary: {summary}"
            )

        except Exception as e:
            logging.error(f"Error summarizing messages: {e}")
            return ChatMessage(role="system", content="Error summarizing conversation.")

    async def start_summary_task(self, messages: List[ChatMessage]):
        """
        Start a new task to summarize the messages.
        """
        try:
            if self._summary_task and not self._summary_task.done():
                logging.info("Previous summary task still running")
                return

            self._summary_task = asyncio.create_task(self.summarize_messages(messages))

            def callback(task):
                try:
                    if not task.cancelled():
                        summary_message = task.result()
                        messages.clear()
                        messages.append(summary_message)
                        logging.info("Successfully summarized and updated history")
                except Exception as e:
                    logging.error(f"Error in summary task callback: {e}")
                    messages.pop(0) if messages else None
                    messages.pop(0) if messages else None

            self._summary_task.add_done_callback(callback)

        except Exception as e:
            logging.error(f"Error starting summary task: {e}")

    def get_messages(self) -> List[dict]:
        """
        Get messages in format required by OpenAI API.
        """
        return [{"role": msg.role, "content": msg.content} for msg in self.history]

    @staticmethod
    def update_history():
        def decorator(func: Callable[..., Awaitable[R]]) -> Callable[..., Awaitable[R]]:
            @functools.wraps(func)
            async def wrapper(self: Any, prompt: str, *args, **kwargs) -> R:
                if self._config.history_length == 0:
                    return await func(self, prompt, [], *args, **kwargs)

                formatted_inputs = f"State Input: {" | ".join(f"{input_type}: {input_info.input}" for input_type, input_info in self.io_provider.inputs.items())}"
                user_message = ChatMessage(role="user", content=formatted_inputs)

                self.history_manager.history.append(user_message)

                messages = self.history_manager.get_messages()
                response = await func(self, prompt, messages, *args, **kwargs)

                if response is not None:
                    assistant_message = "Action Output: " + (
                        " | ".join(
                            ACTION_MAP[command.name].format(
                                command.arguments[0].value if command.arguments else ""
                            )
                            for command in response.commands
                            if command.name in ACTION_MAP
                        )
                    )

                    self.history_manager.history.append(
                        ChatMessage(role="assistant", content=assistant_message)
                    )

                    if (
                        self.history_manager.config.history_length > 0
                        and len(self.history_manager.history)
                        > self.history_manager.config.history_length
                    ):
                        await self.history_manager.start_summary_task(
                            self.history_manager.history
                        )

                return response

            return wrapper

        return decorator
