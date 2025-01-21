import openai
import logging
import typing as T
from dataclasses import dataclass
import time

from pydantic import BaseModel

from llm import LLM

from llm.output_model import Command
from llm.output_model import LLM_full
    
R = T.TypeVar("R")

class OpenAILLM(LLM[R]):

    def __init__(self, output_model: T.Type[R]):
        self._client = openai.AsyncClient()
        super().__init__(output_model)

    async def ask(self, prompt: str, inputs: list[str]) -> R | None:
        try:
            logging.info(f"OpenAILLM prompt: {prompt}")
            parsed_response = await self._client.beta.chat.completions.parse(
                model="gpt-4o",
                messages=[{"role": "user", "content": prompt}],
                response_format=self._output_model,
            )
            message_content = parsed_response.choices[0].message.content

            try:
                parsed_response = self._output_model.model_validate_json(message_content)
                logging.info(f"OpenAILLM response: {parsed_response}")
                
                payload = LLM_full(prompt, inputs, parsed_response, f"{time.time():.3f}")
                logging.info(f"OpenAILLM payload: {payload}")

                return payload
            except Exception as e:
                logging.error(f"Error parsing response: {e}")
                return None

        except Exception as e:
            logging.error(f"Error asking LLM: {e}")
            return None