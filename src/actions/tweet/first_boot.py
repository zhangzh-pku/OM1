import logging
import os
from pathlib import Path
import warnings
from dotenv import load_dotenv
from runtime.config import RuntimeConfig

def send_first_boot_tweet(config: RuntimeConfig):
    """
    Send a 'Hello I'm alive' tweet on first omOS boot.
    
    Parameters
    ----------
    config : RuntimeConfig
        Runtime configuration containing agent name
    """
    # Create .omos directory in project root if it doesn't exist
    project_root = Path(__file__).parent.parent.parent.parent
    omos_dir = project_root / '.omos'
    omos_dir.mkdir(exist_ok=True)
    
    # Check if first boot marker exists
    first_boot_file = omos_dir / 'first_boot_tweet'
    if first_boot_file.exists():
        logging.info("First boot tweet already sent")
        return

    try:
        # Initialize Twitter client
        load_dotenv()
        
        # Suppress tweepy warnings
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=SyntaxWarning)
            import tweepy
            
            client = tweepy.Client(
                consumer_key=os.getenv('TWITTER_API_KEY'),
                consumer_secret=os.getenv('TWITTER_API_SECRET'),
                access_token=os.getenv('TWITTER_ACCESS_TOKEN'),
                access_token_secret=os.getenv('TWITTER_ACCESS_TOKEN_SECRET')
            )
        
        # Get agent name from config
        agent_name = config.name
        
        # Send first boot tweet
        tweet_text = f"🤖 Hello, I'm alive! I'm {agent_name}, an AI agent powered by omOS. Ready to explore and learn! 🌟 #AI #omOS"
        response = client.create_tweet(text=tweet_text)
        
        # Create marker file
        first_boot_file.touch()
        
        # Log success
        tweet_id = response.data['id']
        tweet_url = f"https://twitter.com/user/status/{tweet_id}"
        logging.info(f"First boot tweet sent! URL: {tweet_url}")
        
    except Exception as e:
        logging.error(f"Failed to send first boot tweet: {str(e)}")