import tweepy
import logging
from typing import Optional
from .singleton import singleton
from rag_provider import RAGProvider

@singleton
class TwitterProvider:
    """Provider for Twitter interactions"""
    
    def __init__(self):
        self.client = None
        self.rag = RAGProvider()
    
    def connect(self):
        """Initialize Twitter client"""
        if not self.client:
            self.client = tweepy.Client(
                consumer_key=os.getenv('TWITTER_API_KEY'),
                consumer_secret=os.getenv('TWITTER_API_SECRET'),
                access_token=os.getenv('TWITTER_ACCESS_TOKEN'),
                access_token_secret=os.getenv('TWITTER_ACCESS_TOKEN_SECRET')
            )
    
    async def tweet_rag_result(self, query: str):
        """Query RAG and tweet results"""
        try:
            self.connect()
            results = await self.rag.query(query)
            if results:
                tweet = f"Q: {query}\nA: {results[0]['text'][:200]}..."
                self.client.create_tweet(text=tweet)
                return True
        except Exception as e:
            logging.error(f"Failed to tweet RAG result: {e}")
            return False