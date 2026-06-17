"""
Base hardware-driver abstraction.

The portfolio project uses this base class to demonstrate a clean driver
lifecycle for camera hardware integrations.
"""

from abc import ABC, abstractmethod
from threading import Lock

from .driver_exception import DriverNotConnectedError


class BaseDriver(ABC):
    """Unified lifecycle and thread-safety helper for hardware drivers."""

    def __init__(self):
        self._connected = False
        self._lock = Lock()

    @abstractmethod
    def connect(self) -> None:
        """Establish hardware connection."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close hardware connection."""

    def is_connected(self) -> bool:
        return self._connected

    def health_check(self) -> bool:
        """Default health check returns connection state."""
        return self._connected

    def _require_connection(self) -> None:
        if not self._connected:
            raise DriverNotConnectedError(f"{self.__class__.__name__} is not connected.")
