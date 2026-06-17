# Driver Architecture

The camera driver follows a layered structure:

```text
RSCamDriver
    ↓
RSPipeline
    ↓
RealSense SDK / pyrealsense2
    ↓
Depth + Color frames
    ↓
rs_processing.create_pointcloud()
```

## Reliability Features

- connection state tracking
- lock-based thread safety
- unified connect and disconnect lifecycle
- custom driver exceptions
- stream profile validation
- safe stop and cleanup

## Why This Matters

Industrial camera drivers should not expose raw hardware calls directly to the application layer. A driver wrapper improves reliability, testing, error handling, and future replacement of camera hardware.
