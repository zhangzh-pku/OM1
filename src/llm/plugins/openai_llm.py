import openai
import typing as T

from pydantic import BaseModel

from llm import LLM


R = T.TypeVar("R", bound=BaseModel)


class OpenAILLM(LLM[R]):
    def __init__(self, output_model: T.Type[R]):
        self._client = openai.AsyncClient()
        super().__init__(output_model)

    async def ask(self, prompt: str) -> R | None:
        parsed_response = await self._client.beta.chat.completions.parse(
            model="gpt-4o",
            messages=[{"role": "user", "content": prompt}],
            response_format=self._output_model,
        )
        return parsed_response.choices[0].message.parsed
