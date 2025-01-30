from actions.base import ActionImplementation
from actions.tweet.interface import TweetInput


class TweetPassthrough(ActionImplementation[TweetInput, TweetInput]):
    """Simple passthrough implementation for tweet action."""

    async def execute(self, interface: TweetInput) -> TweetInput:
        """Execute the tweet action."""
        return interface  # Pass through the input interface as output
