from unittest.mock import MagicMock, Mock, patch

import pytest

from providers.singleton import singleton
from providers.tts_provider import TTSProvider


@pytest.fixture(autouse=True)
def reset_singleton():
    singleton.instances = {}
    yield


@pytest.fixture(autouse=True)
def mock_pyaudio():
    with patch("pyaudio.PyAudio") as mock:
        mock_instance = MagicMock()
        mock_instance.get_default_output_device_info.return_value = {
            "name": "Mock Speaker",
            "index": 0,
        }
        mock.return_value = mock_instance
        yield mock


@pytest.fixture(autouse=True)
def mock_audio_stream():
    with patch("providers.tts_provider.AudioOutputStream") as mock:
        yield mock


@pytest.fixture
def tts_provider(mock_audio_stream):
    provider = TTSProvider(url="test_url")
    yield provider
    provider.stop()


def test_singleton_behavior():
    provider1 = TTSProvider(url="test_url")
    provider2 = TTSProvider(url="another_url")
    assert provider1 is provider2


def test_initialization(mock_audio_stream, mock_pyaudio):
    provider = TTSProvider(url="test_url")
    assert provider.running is False
    assert provider._thread is None
    mock_audio_stream.assert_called_once_with(url="test_url", device=0)


def test_start_stop(mock_audio_stream):
    provider = TTSProvider(url="test_url")
    provider.start()
    assert provider.running is True
    assert provider._thread is not None
    assert provider._thread.is_alive()

    provider.stop()
    assert provider.running is False
    # Wait a short time to ensure thread cleanup
    provider._thread.join(timeout=1)
    assert not provider._thread.is_alive()


def test_register_callback(mock_audio_stream):
    provider = TTSProvider(url="test_url")
    callback = Mock()
    provider.register_tts_state_callback(callback)
    mock_audio_stream.return_value.set_tts_state_callback.assert_called_once_with(
        callback
    )


def test_add_pending_message(mock_audio_stream):
    provider = TTSProvider(url="test_url")
    provider.add_pending_message("test message")
    mock_audio_stream.return_value.add.assert_called_once_with("test message")


def test_multiple_start_calls(mock_audio_stream):
    provider = TTSProvider(url="test_url")
    provider.start()
    initial_thread = provider._thread
    provider.start()
    assert provider._thread is initial_thread
