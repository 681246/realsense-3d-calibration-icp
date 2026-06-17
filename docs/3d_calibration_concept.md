# 3D Calibration Concept

## Goal

The goal of 3D calibration is to compare a current scan against a trained master point cloud and compute the spatial difference.

The result is a 4x4 transformation matrix that describes how the current scan must move to align with the master scan.

## Concept

Current PCD and master PCD are passed into Open3D ICP registration. The output is a transformation matrix. From this matrix, the software extracts translation and rotation values.

## Output Values

| Value | Meaning |
|---|---|
| `tx` | X translation difference |
| `ty` | Y translation difference |
| `tz` | Z translation difference |
| `rx` | X rotation difference |
| `ry` | Y rotation difference |
| `rz` | Z rotation difference |

## Quality Metrics

| Metric | Meaning |
|---|---|
| `fitness` | How much of the scan aligns with the reference |
| `inlier_rmse` | Alignment error among matching points |
| `identity_transform` | Whether ICP returned almost no movement |
