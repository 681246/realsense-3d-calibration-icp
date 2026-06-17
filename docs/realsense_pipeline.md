# RealSense Pipeline

The RealSense pipeline handles the depth and color camera lifecycle.

## Responsibilities

1. depth stream configuration
2. color stream configuration
3. pipeline start and stop
4. optional advanced camera config loading
5. exposure setup
6. depth scale reading
7. depth to color alignment
8. frame profile validation

## Industrial Importance

For production inspection, the captured depth and color frames must match the configured stream profile. A mismatch can create incorrect point-cloud scale or unstable calibration results.

## Reliability Notes

The pipeline should fail fast when:

- camera is not connected
- frame capture fails
- depth or color frame is missing
- captured stream profile does not match configured profile
- RealSense config file cannot be loaded
