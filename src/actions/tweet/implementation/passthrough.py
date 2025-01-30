from actions.base import ActionImplementation
from actions.tweet.interface import TweetInput


class TweetPassthrough(ActionImplementation[TweetInput]):
    """Simple passthrough implementation for tweet action"""

    async def execute(self, interface: TweetInput) -> None:
        """Execute the tweet action"""
        pass