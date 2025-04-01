import logging
import threading
import time
from queue import Queue
from typing import Callable, Optional

from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.topic import Topic
from providers.cyclonedds_writer_provider import SpeechData


class CycloneDDSReaderProvider:
    """
    Reader provider for reading messages using CycloneDDS.

    This class manages the CycloneDDS DomainParticipant, Topic, DataReader, a message queue, and a worker thread that
    continuously listens to messages to a specified topic.
    """

    def __init__(self, message_buffer: Queue = None, message_callback: Optional[Callable] = None, topic: str = "speech"):
        """
        Initialize the CycloneDDS reader provider and the DomainParticipant, Topic, DataReader

        Parameters
        ----------
        topic : str, optional
            The topic on which to subscribe messages (default is "speech").
        """
        try:
            self.dp = DomainParticipant()
            logging.info("CycloneDDS DomainParticipant created (Reader)")
        except Exception as e:
            logging.error(f"Error creating CycloneDDS DomainParticipant (Reader): {e}")
            self.dp = None

        try:
            self.tp = Topic(self.dp, topic, SpeechData)
            logging.info("CycloneDDS Topic created (Reader)")
        except Exception as e:
            logging.error(f"Error creating CycloneDDS Topic (Reader): {e}")
            self.tp = None

        # try:
        #     self.listener = MyListener()
        #     if message_callback is not None:
        #         self.listener.set_on_data_available(message_callback)
        #     logging.info("CycloneDDS Listener created")
        # except Exception as e:
        #     logging.error(f"Error creating CycloneDDS Listener: {e}")
        #     self.listener = None

        try:
            self.dr = DataReader(self.dp, self.tp)
            logging.info("CycloneDDS DataReader created")
        except Exception as e:
            logging.error(f"Error creating CycloneDDS DataReader: {e}")
            self.dr = None

        self.message_callback = message_callback

        # Pending message queue and threading constructs
        self._pending_messages = Queue()
        self._lock = threading.Lock()
        self.running: bool = False
        self._thread: Optional[threading.Thread] = None

    def read_message(self):
        """
        Function for processing incoming messages.
        """
        message = None
        if self.dr is not None:
            read_data = self.dr.take_next()
            if read_data is not None:
                message = read_data.data
        else:
            logging.error("No Data Read. Cannot read CycloneDDS message.")
        return message

    def start(self):
        """
        Start the reader provider by launching the background thread.
        """
        if self.running:
            return

        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logging.info("CycloneDDS Reader Provider started")

    def _run(self):
        """
        Internal method to run the provider's main processing loop.

        This method runs in a separate thread and handles the continuous processing
        of zenoh messages.
        """
        while self.running:
            try:
                time.sleep(0.1)
                heard_message = self.read_message()
                if heard_message is not None and self.message_callback is not None:
                    self.message_callback(heard_message)
            except Exception as e:
                logging.error(f"CycloneDDS Reader Provider error: {e}")

    def stop(self):
        """
        Stop the reader provider and clean up resources.

        Stops the background thread and closes the Zenoh session.

        Notes
        -----
        The thread join operation uses a 5-second timeout to prevent hanging.
        """
        self.running = False
        if self._thread:
            self._thread.join(timeout=5)
