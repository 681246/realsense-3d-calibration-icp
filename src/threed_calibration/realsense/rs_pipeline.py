"""
Intel RealSense pipeline wrapper.

This module is written as a hardware integration example. It requires a connected
Intel RealSense camera and pyrealsense2 runtime.
"""

import json
import os
import time

import pyrealsense2 as rs

from ..driver_exception import DriverConnectionError


class RSPipeline:
    def __init__(self, width=848, height=480, config_file="config_3.json"):
        self._width = width
        self._height = height
        self._config_file = config_file
        self._pipeline = None
        self._align = None
        self._depth_scale = None
        self._intrinsics = None

    def set_config_file(self, config_file):
        self._config_file = str(config_file or "").strip()

    def set_stream_profile(self, width=None, height=None):
        if width:
            self._width = int(width)
        if height:
            self._height = int(height)

    def start(self):
        start_entered_at = time.monotonic()
        try:
            self._pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, 30)

            profile = self._pipeline.start(config)
            device = profile.get_device()

            if self._config_file and os.path.exists(self._config_file):
                try:
                    advnc_mode = rs.rs400_advanced_mode(device)
                    with open(self._config_file, "r", encoding="utf-8") as f:
                        json_text = f.read()
                    advnc_mode.load_json(json_text)
                except Exception as exc:
                    raise DriverConnectionError(f"3D config file load error: {exc}") from exc

            depth_sensor = device.first_depth_sensor()
            try:
                depth_sensor.set_option(rs.option.exposure, 7000)
            except Exception:
                pass

            self._depth_scale = depth_sensor.get_depth_scale()
            self._align = rs.align(rs.stream.color)

        except Exception as exc:
            duration = time.monotonic() - start_entered_at
            raise DriverConnectionError(f"RealSense start failed after {duration:.3f}s: {exc}") from exc

    def stop(self):
        if self._pipeline is None:
            return
        try:
            self._pipeline.stop()
        except Exception as exc:
            raise DriverConnectionError(str(exc)) from exc
        finally:
            self._pipeline = None
            self._align = None
            self._depth_scale = None
            self._intrinsics = None

    def get_aligned_frames(self):
        if self._pipeline is None or self._align is None:
            raise DriverConnectionError("RealSense pipeline is not started")

        try:
            frames = self._pipeline.wait_for_frames()
            aligned_frames = self._align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                raise DriverConnectionError("Failed to capture aligned frames")

            self._intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

            depth_profile = depth_frame.profile.as_video_stream_profile()
            color_profile = color_frame.profile.as_video_stream_profile()

            if (
                depth_profile.width() != self._width
                or depth_profile.height() != self._height
                or color_profile.width() != self._width
                or color_profile.height() != self._height
            ):
                raise DriverConnectionError(
                    "RealSense profile mismatch: "
                    f"configured={self._width}x{self._height}, "
                    f"depth={depth_profile.width()}x{depth_profile.height()}, "
                    f"color={color_profile.width()}x{color_profile.height()}"
                )

            return depth_frame, color_frame
        except Exception as exc:
            raise DriverConnectionError(str(exc)) from exc

    def get_intrinsics(self):
        return self._intrinsics

    def get_depth_scale(self):
        return self._depth_scale
