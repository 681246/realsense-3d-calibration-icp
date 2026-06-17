"""
Driver exception model for the 3D vision calibration demo.
"""


class DriverError(Exception):
    """Base class for all driver-level errors."""


class DriverConnectionError(DriverError):
    """Raised when connection to hardware fails."""


class DriverTimeoutError(DriverError):
    """Raised when a hardware operation times out."""


class DriverProtocolError(DriverError):
    """Raised when protocol communication fails or data is malformed."""


class DriverNotConnectedError(DriverError):
    """Raised when an operation is attempted without an active connection."""
