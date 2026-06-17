"""
RealSense frame processing helpers.

Converts aligned RealSense depth/color frames into an Open3D RGBD point cloud.
"""

import numpy as np
import open3d as o3d


def create_pointcloud(depth_frame, color_frame, intrinsics, depth_scale):
    """Create image and point cloud artifacts from RealSense frames."""
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    o3d_depth = o3d.geometry.Image(depth_image)
    o3d_color = o3d.geometry.Image(color_image)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color,
        o3d_depth,
        depth_scale=1.0 / depth_scale,
        convert_rgb_to_intensity=False,
    )

    o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width,
        intrinsics.height,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.ppx,
        intrinsics.ppy,
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, o3d_intrinsics)
    return {"image": color_image, "pcd": pcd}
