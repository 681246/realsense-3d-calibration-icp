# Point Cloud Registration

## RGBD to Point Cloud

The RealSense depth and color frames are converted into Open3D images, then combined into an RGBD image. Open3D camera intrinsics are used to create a 3D point cloud.

## ICP Registration

ICP registration aligns a source/current point cloud to a target/master point cloud.

Supported concepts:

- point-to-point ICP
- point-to-plane ICP direction
- CPU registration path
- GPU registration concept from the original reference implementation
- fitness and inlier RMSE based validation

## Recommended Production Validation

- reject empty point clouds
- reject missing PCD paths
- reject low ICP fitness
- reject high RMSE
- flag suspicious identity transform
- log source and target point counts
