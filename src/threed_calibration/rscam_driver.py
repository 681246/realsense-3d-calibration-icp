"""
RealSense camera driver.

Provides:
- connection lifecycle
- single 3D grab
- optional 2D color streaming
- thread-safe pipeline access
"""

import logging
import threading

import numpy as np

from .base_driver import BaseDriver
from .driver_exception import DriverConnectionError, DriverProtocolError
from .realsense.rs_pipeline import RSPipeline
from .realsense.rs_processing import create_pointcloud


class RSCamDriver(BaseDriver):
    def __init__(self, width=848, height=480, config_file="config_3.json"):
        super().__init__()
        self._pipeline = RSPipeline(width=width, height=height, config_file=config_file)
        self._latest_frame = None
        self._streaming = False
        self._stream_thread = None
        self._logger = logging.getLogger("RSCamDriver")

    def set_logger(self, logger):
        self._logger = logger

    def set_config_file(self, config_file):
        self._pipeline.set_config_file(config_file)

    def set_stream_profile(self, width=None, height=None):
        self._pipeline.set_stream_profile(width=width, height=height)

    def connect(self):
        try:
            self._pipeline.start()
            self._connected = True
            self._logger.info("RealSense pipeline started")
        except Exception as exc:
            self._connected = False
            raise DriverConnectionError(str(exc)) from exc

    def disconnect(self):
        try:
            self.stop_streaming()
            self._pipeline.stop()
            self._connected = False
            self._logger.info("RealSense pipeline stopped")
        except Exception as exc:
            self._connected = False
            raise DriverConnectionError(str(exc)) from exc

    def grab(self):
        self._require_connection()
        try:
            with self._lock:
                depth_frame, color_frame = self._pipeline.get_aligned_frames()
                intrinsics = self._pipeline.get_intrinsics()
                depth_scale = self._pipeline.get_depth_scale()
            return create_pointcloud(depth_frame, color_frame, intrinsics, depth_scale)
        except Exception as exc:
            raise DriverProtocolError(str(exc)) from exc

    def start_streaming(self, callback=None):
        self._require_connection()
        if self._streaming:
            return

        self._streaming = True

        def _stream_loop():
            while self._streaming:
                try:
                    with self._lock:
                        _depth_frame, color_frame = self._pipeline.get_aligned_frames()
                    frame = np.asanyarray(color_frame.get_data())
                    with self._lock:
                        self._latest_frame = frame
                    if callback is not None:
                        callback(frame)
                except Exception as exc:
                    self._logger.error("RealSense streaming error: %s", exc)
                    self._streaming = False
                    break

        self._stream_thread = threading.Thread(target=_stream_loop, daemon=True)
        self._stream_thread.start()

    def stop_streaming(self):
        if not self._streaming:
            return
        self._streaming = False
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None

    def get_latest_frame(self):
        with self._lock:
            return self._latest_frame
