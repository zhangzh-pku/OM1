import threading
from typing import Any, Dict

def singleton(cls):
    """
    A thread-safe singleton decorator that ensures only one instance of a class exists.

    This decorator implements the singleton pattern with thread safety using a lock.
    Multiple threads attempting to create an instance will be synchronized to prevent
    race conditions.

    Args:
        cls: The class to be converted into a singleton.

    Returns:
        function: A getter function that returns the singleton instance.
    """
    instances = {}
    lock = threading.Lock()

    def get_instance(*args, **kwargs) -> Any:
        """
        Returns the singleton instance of the decorated class.

        If the instance doesn't exist, creates it with the provided arguments.
        Thread-safe implementation using a lock.

        Args:
            *args: Positional arguments to pass to the class constructor.
            **kwargs: Keyword arguments to pass to the class constructor.

        Returns:
            Any: The singleton instance of the decorated class.
        """
        with lock:
            if cls not in instances:
                instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance

@singleton
class GlobalState:
    """
    A thread-safe global state manager implemented as a singleton.

    This class provides a centralized way to store and manage global state in a
    thread-safe manner. It uses a dictionary to store key-value pairs and implements
    thread safety using locks.

    The class is decorated with @singleton to ensure only one instance exists
    across the application.
    """
    def __init__(self):
        """
        Initializes the GlobalState with an empty state dictionary and a thread lock.
        """
        self._state: Dict[str, Any] = {}
        self._lock = threading.Lock()

    def set(self, key: str, value: Any) -> None:
        """
        Sets a value in the global state with thread safety.

        Args:
            key (str): The key under which to store the value.
            value (Any): The value to store.
        """
        with self._lock:
            self._state[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        """
        Retrieves a value from the global state with thread safety.

        Args:
            key (str): The key of the value to retrieve.
            default (Any, optional): The value to return if the key doesn't exist.
                Defaults to None.

        Returns:
            Any: The value associated with the key, or the default value if the
                key doesn't exist.
        """
        with self._lock:
            return self._state.get(key, default)

    def clear(self) -> None:
        """
        Clears all values from the global state with thread safety.
        """
        with self._lock:
            self._state.clear()