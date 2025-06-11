import logging
import threading
import time
import requests
import json
from typing import Callable, Optional, Dict
from .singleton import singleton


@singleton
class UbtechASRProvider:
    _instance = None

    @staticmethod
    def get_instance():
        """Returns the singleton instance of the provider."""
        return UbtechASRProvider._instance

    def __init__(self, robot_ip: str, language_code: str = 'en'):
        UbtechASRProvider._instance = self
        
        self.robot_ip = robot_ip
        self.language = language_code  # Add language attribute

        self.basic_url = f"http://{self.robot_ip}:9090/v1/"
        self.headers = {'Content-Type': 'application/json'}
        self.session = requests.Session() # Create a session object
        self.session.headers.update(self.headers) # Update session headers

        self.running = False
        self.paused = False # Add paused flag
        self._thread: Optional[threading.Thread] = None
        self._message_callback: Optional[Callable] = None
        self._set_robot_language(language_code)

    def register_message_callback(self, cb: Optional[Callable]):
        self._message_callback = cb

    def start(self):
        if self.running: return
        logging.info("Starting UbtechASRProvider background thread...")
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        if not self.running: return
        logging.info("Stopping UbtechASRProvider background thread...")
        self.running = False
        self._stop_voice_iat()
        if self._thread: self._thread.join(timeout=3)
        logging.info("UbtechASRProvider stopped.")

    def pause(self):
        logging.debug("Pausing UbtechASRProvider")
        self.paused = True

    def resume(self):
        logging.debug("Resuming UbtechASRProvider")
        self.paused = False

    def _run(self):
        while self.running:
            if self.paused:
                time.sleep(0.1) # Sleep while paused
                continue

            try:
                logging.debug("UbtechASRProvider: _run loop iteration.")
                text = self._get_single_utterance()
                if text and self._message_callback:
                    logging.debug(f"UbtechASRProvider: Calling message callback with: '{text}'")
                    self._message_callback(text)
                    self.pause() # Automatically pause after sending a message
                time.sleep(1.0) # Slightly increased sleep time for potentially less aggressive polling
            except Exception as e:
                logging.error(f"UbtechASRProvider: Error in _run loop: {e}", exc_info=True)
                time.sleep(5) # Longer sleep on error

    def _get_single_utterance(self) -> Optional[str]:
        ts = int(time.time())
        logging.debug(f"UbtechASRProvider: _get_single_utterance called, timestamp: {ts}")
        
        session_started = False
        try:
            if not self._start_voice_iat(ts):
                logging.debug("UbtechASRProvider: _start_voice_iat failed or returned False.")
                return None
            session_started = True # Mark that ASR session was started on the robot
            logging.debug("UbtechASRProvider: _start_voice_iat successful.")
            
            for i in range(100): # Poll for ~10 seconds (100 * 0.1s)
                if not self.running: 
                    logging.debug("UbtechASRProvider: Not running, exiting _get_single_utterance loop.")
                    return None
                res = self._get_voice_iat()
                logging.debug(f"UbtechASRProvider: _get_voice_iat (attempt {i+1}) returned: {res}")
                if res.get("status") == "idle" and res.get("timestamp") == ts:
                    if not res.get("data") or res.get("code") != 0: 
                        logging.debug("UbtechASRProvider: Idle status but no data or error code.")
                        return None
                    words = res.get("data", {}).get("text", {}).get("ws", [])
                    processed_text = "".join(w['cw'][0]['w'] for w in words).strip().lower() if words else None
                    logging.debug(f"UbtechASRProvider: Processed text from idle status: '{processed_text}'")
                    # No need to stop here, as idle means it's ready for next start or has finished.
                    # The finally block will handle stopping if this utterance cycle is truly over.
                    return processed_text
                time.sleep(0.1)
            logging.debug("UbtechASRProvider: _get_single_utterance loop finished without idle status (timeout).")
            return None # Timed out polling for this utterance
        finally:
            if session_started:
                # If an ASR session was successfully started on the robot for this utterance attempt,
                # ensure we try to stop it to clean up resources on the robot.
                logging.debug("UbtechASRProvider: Ensuring voice iat session is stopped.")
                self._stop_voice_iat()

    def _set_robot_language(self, lang_code: str):
        try:
            logging.info(f"Setting robot language to: {lang_code}")
            self.session.put(f"{self.basic_url}system/language", data=json.dumps({"language": lang_code}), timeout=3) # Use session
        except requests.RequestException as e:
            logging.error(f"UbtechASRProvider: Failed to set robot language: {e}")

    def _start_voice_iat(self, ts: int) -> bool:
        if not self.robot_ip:
            logging.error("Robot IP not set, cannot start ASR session.")
            return False
        try:
            # Try to set language (assuming 'lang' is the parameter name)
            data = {"text": "", "timestamp": ts, "lang": self.language}
            logging.debug(f"Starting ASR session with timestamp {ts} and language {self.language}. Payload: {data}")
            res = self.session.put(f"{self.basic_url}voice/iat", json=data, timeout=3) # Use session
            res.raise_for_status() # Raise HTTPError for bad responses (4xx or 5xx)
            response_json = res.json()
            logging.debug(f"UbtechASRProvider: _start_voice_iat response: {response_json}")
            return response_json.get("code") == 0
        except requests.RequestException as e: 
            logging.error(f"UbtechASRProvider: _start_voice_iat request failed: {e}")
            return False

    def _stop_voice_iat(self):
        try: 
            logging.debug("UbtechASRProvider: Attempting to stop voice iat.")
            # For DELETE, it's common not to have a body or need specific content-type if API allows
            self.session.delete(f"{self.basic_url}voice/iat", timeout=2) # Use session
            logging.debug("UbtechASRProvider: _stop_voice_iat request sent.")
        except requests.RequestException as e: 
            logging.error(f"UbtechASRProvider: _stop_voice_iat request failed: {e}")
            pass

    def _get_voice_iat(self) -> Dict:
        try:
            logging.debug("UbtechASRProvider: Attempting to get voice iat.")
            res = self.session.get(f"{self.basic_url}voice/iat", timeout=3) # Use session
            res.raise_for_status()
            response_json = res.json()
            logging.debug(f"UbtechASRProvider: _get_voice_iat raw response: {response_json}")
            # The original cleaning logic for data field if it's an embedded JSON string
            if response_json.get("data") and isinstance(response_json["data"], str) and response_json.get("code") == 0:
                cleaned_data_str = response_json["data"].strip().rstrip('\x00')
                logging.debug(f"UbtechASRProvider: _get_voice_iat cleaned data string: '{cleaned_data_str}'")
                if cleaned_data_str: 
                    response_json["data"] = json.loads(cleaned_data_str)
                else:
                    response_json["data"] = {} 
            return response_json
        except requests.RequestException as e: 
            logging.error(f"UbtechASRProvider: _get_voice_iat request failed: {e}")
            return {"status": "error_request", "code": -1} # Ensure a code for error checks
        except json.JSONDecodeError as e:
            logging.error(f"UbtechASRProvider: _get_voice_iat JSON decode failed: {e}. Raw data: '{res.text if 'res' in locals() else 'N/A'}'")
            return {"status": "error_json_decode", "code": -1}