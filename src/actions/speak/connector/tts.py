from actions.base import ActionConnector
from actions.speak.interface import SpeakInput
from providers.asr_provider import ASRProvider
from providers.tts_provider import TTSProvider


class SpeakRos2Connector(ActionConnector[SpeakInput]):
    def __init__(self):
        super().__init__()

        # Initialize ASR and TTS providers
        self.asr = ASRProvider(ws_url="wss://api-asr.openmind.org")
        self.tts = TTSProvider(url="https://api-tts.openmind.org")
        self.tts.start()

    async def connect(self, output_interface: SpeakInput) -> None:
        # Block ASR until TTS is done
        self.tts.register_tts_state_callback(self.asr.audio_stream.on_tts_state_change)
        # Add pending message to TTS
        self.tts.add_pending_message(output_interface.sentence)
