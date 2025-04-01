import json
import logging
import threading
import time
from queue import Empty, Queue
from typing import Optional

from dataclasses import dataclass
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.topic import Topic
from cyclonedds.idl import IdlStruct


# Define a message datatype with one member, data, with as type 'string'
@dataclass
class SpeechData(IdlStruct, typename="SpeechData.Msg"):
    data: str


class CycloneDDSWriterProvider:
    """
    Writer provider for sending messages using a CycloneDDS session

    This class manages a CycloneDDS DomainParticipant, Topic, DataWriter, a message queue, and a worker thread that
    continuously publishes queued messages to a specified topic.
    """

    def __init__(self, topic: str = "speech"):
        """
        Initialize the CycloneDDS publisher provider and create a CycloneDDS DomainParticipant, Topic, and DataWriter.

        Parameters
        ----------
        topic : str, optional
            The topic on which to publish messages (default is "speech").
        """
        try:
            self.dp = DomainParticipant()
            logging.info("CycloneDDS DomainParticipant created (Writer)")
        except Exception as e:
            logging.error(f"Error creating CycloneDDS DomainParticipant (Writer): {e}")
            self.dp = None

        try:
            self.tp = Topic(self.dp, topic, SpeechData)
            logging.info("CycloneDDS Topic created (Writer)")
        except Exception as e:
            logging.error(f"Error creating CycloneDDS Topic (Writer): {e}")
            self.tp = None

        try:
            self.dw = DataWriter(self.dp, self.tp)
            logging.info("CycloneDDS DataWriter created")
        except Exception as e:
            logging.error(f"Error creating CycloneDDS DataWriter: {e}")
            self.dw = None

        # Pending message queue and threading constructs
        self._pending_messages = Queue()
        self._lock = threading.Lock()
        self.running: bool = False
        self._thread: Optional[threading.Thread] = None

    def add_pending_message(self, text: str):
        """
        Create a SpeechData message from the provided text and add it to the queue.

        Parameters
        ----------
        text : str
            The textual content to be published as a message.
        """
        msg = SpeechData(data=text)
        logging.info(f"Queueing message: {text}")
        self._pending_messages.put(msg)

    def _publish_message(self, msg: SpeechData):
        """
        Publish a single SpeechData message using CycloneDDS.

        Parameters
        ----------
        msg : SpeechData
            The message to be published.
        """
        # Attempt to write the message with a timeout of 0.5 seconds

        if self.dw is None:
            logging.info("No open CycloneDDS DataWriter, returning")
            return
        logging.info("Publishing message: {} ".format(msg))
        self.dw.write(msg)

    def start(self):
        """
        Start the publisher provider by launching the processing thread.
        """
        if self.running:
            return

        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logging.info("CycloneDDS Writer Provider started")

    def _run(self):
        """
        Internal loop that processes and publishes pending messages.
        """
        while self.running:
            try:
                # Wait up to 0.5 seconds for a message.
                msg = self._pending_messages.get(timeout=0.5)
                self._publish_message(msg)
            except Empty:
                continue
            except Exception as e:
                logging.exception("Exception in CycloneDDS Writer thread: %s", e)

    def stop(self):
        """
        Stop the writer provider and clean up resources.
        """
        self.running = False
        if self._thread:
            self._thread.join(timeout=5)
        logging.info("CycloneDDS Writer Provider stopped")
