import logging

import pyaudio

from actions.base import ActionConnector
from actions.speak.interface import SpeakInput
from providers.asr_provider import ASRProvider
from providers.tts_provider import TTSProvider


class SpeakRos2Connector(ActionConnector[SpeakInput]):

    def __init__(self):
        super().__init__()

        p = pyaudio.PyAudio()
        info = p.get_host_api_info_by_index(0)
        numdevices = info.get("deviceCount")

        # i do not know how to access configuration info from an action
        # self.config = {
        #     "speaker_name": "USB PnP Audio Device",
        #     "microphone_name": "USB PnP Audio Device"
        # }

        class config_dummy:
            speaker_name = "USB PnP Audio Device"
            microphone_name = "USB PnP Audio Device"

        self.config = config_dummy()

        # self.config["speaker_name"] = "USB PnP Audio Device"
        # self.config["microphone_name"] = "USB PnP Audio Device"

        # for i in range(0, numdevices):
        #     dev = p.get_device_info_by_host_api_device_index(0, i)
        #     name = dev.get("name")
        #     channels = dev.get("maxInputChannels")
        #     logging.info(f"TTS.PY: Audio device '{name}' at ID:{i} with {channels} InputChannels")

        mic_device_id = None
        if hasattr(self.config, "microphone_name"):
            microphone_name = self.config.microphone_name
            logging.info(f"TTS.PY: looking for microphone '{microphone_name}'")
            for i in range(0, numdevices):
                dev = p.get_device_info_by_host_api_device_index(0, i)
                if (dev.get("maxInputChannels")) > 0:  # this is a microphone
                    if microphone_name in dev.get("name"):
                        mic_device_id = i
                        logging.info(
                            f"TTS.PY: Found microphone specified in .json as '{microphone_name}' at ID:{mic_device_id}"
                        )
                        break

        speaker_device_id = None
        if hasattr(self.config, "speaker_name"):
            speaker_name = self.config.speaker_name
            logging.info(f"TTS.PY: looking for speaker '{speaker_name}'")
            for i in range(0, numdevices):
                dev = p.get_device_info_by_host_api_device_index(0, i)
                if (dev.get("maxOutputChannels")) > 0:  # this is a speaker
                    if speaker_name in dev.get("name"):
                        speaker_device_id = i
                        logging.info(
                            f"TTS.PY: Found speaker specified in .json as '{speaker_name}' at ID:{speaker_device_id}"
                        )
                        break

        # Initialize ASR and TTS providers
        self.asr = ASRProvider(
            ws_url="wss://api-asr.openmind.org", device_id=mic_device_id
        )
        self.tts = TTSProvider(
            url="https://api-tts.openmind.org", device_id=speaker_device_id
        )
        self.tts.start()

    async def connect(self, output_interface: SpeakInput) -> None:
        # Block ASR until TTS is done
        self.tts.register_tts_state_callback(self.asr.audio_stream.on_tts_state_change)
        # Add pending message to TTS
        self.tts.add_pending_message(output_interface.sentence)
